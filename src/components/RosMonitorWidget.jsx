import React, { useEffect, useState, useRef } from 'react';
import rosInit from '../ros/rosbridge';
import ROSLIB from 'roslib';
import githubLogo from '../assets/github-mark.svg';
import { useRef as chartRef, useEffect as chartEffect } from 'react';
import { Chart, LineController, LineElement, PointElement, LinearScale, Title, CategoryScale } from 'chart.js';
import * as d3 from 'd3';
Chart.register(LineController, LineElement, PointElement, LinearScale, Title, CategoryScale);

function toHex(val) {
  if (typeof val === 'number') {
    return '0x' + val.toString(16);
  }
  if (typeof val === 'string') {
    // Convert each char to hex
    return val.split('').map(c => c.charCodeAt(0).toString(16).padStart(2, '0')).join(' ');
  }
  if (Array.isArray(val)) {
    return '[' + val.map(toHex).join(', ') + ']';
  }
  if (val instanceof ArrayBuffer || ArrayBuffer.isView(val)) {
    const arr = new Uint8Array(val.buffer || val);
    return Array.from(arr).map(b => b.toString(16).padStart(2, '0')).join(' ');
  }
  if (typeof val === 'object' && val !== null) {
    // Recursively convert object
    let out = '{ ';
    for (const k in val) {
      out += `${k}: ${toHex(val[k])}, `;
    }
    return out + '}';
  }
  return String(val);
}

// New component to render message structure
function MessageRenderer({ data, showHex }) {
  if (data === null || typeof data === 'undefined') {
    return <span style={{ color: '#888' }}>null</span>;
  }

  if (typeof data !== 'object' || data === null) {
    return <span>{showHex ? toHex(data) : String(data)}</span>;
  }

  if (Array.isArray(data)) {
    // For large arrays, we might want to just show a summary
    if (data.length > 10) {
      return <span style={{ color: '#888' }}>[Array length: {data.length}]</span>;
    }
    return (
      <div style={{ paddingLeft: '1em' }}>
        [
        {data.map((item, index) => (
          <div key={index}>
            <MessageRenderer data={item} showHex={showHex} />,
          </div>
        ))}
        ]
      </div>
    );
  }

  return (
    <table style={{ borderCollapse: 'collapse', width: '100%' }}>
      <tbody>
        {Object.entries(data).map(([key, value]) => (
          <tr key={key} style={{ border: '1px solid #444' }}>
            <td style={{ padding: '4px', fontWeight: 'bold', verticalAlign: 'top', width: '150px' }}>{key}</td>
            <td style={{ padding: '4px', verticalAlign: 'top' }}>
              <MessageRenderer data={value} showHex={showHex} />
            </td>
          </tr>
        ))}
      </tbody>
    </table>
  );
}

function RosMonitorWidget({ panelId, host, viewMode: initialViewMode }) {
  const [topics, setTopics] = useState([]);
  const [loading, setLoading] = useState(true);
  const [error, setError] = useState(null);
  const [messages, setMessages] = useState([]);
  const [latestMessage, setLatestMessage] = useState(null);
  const [listener, setListener] = useState(null);
  const [filter, setFilter] = useState('');
  const [showHex, setShowHex] = useState(false);
  const timestampsRef = useRef([]); // Store timestamps of received messages
  const [isConnected, setIsConnected] = useState(false);
  const [viewMode, setViewMode] = useState(initialViewMode === 'nodes' ? 'nodes' : 'messages');
  const chartCanvasRef = chartRef(null);
  const chartInstanceRef = chartRef(null);
  const [numericFields, setNumericFields] = useState([]);
  const [selectedField, setSelectedField] = useState('');
  const [messageType, setMessageType] = useState('');
  const [selectedTopic, setSelectedTopic] = useState('');
  const [nodes, setNodes] = useState([]);
  const [nodeTopics, setNodeTopics] = useState([]);
  const [publishers, setPublishers] = useState({});
  const [subscribers, setSubscribers] = useState({});
  const [services, setServices] = useState([]);
  const [serviceNodes, setServiceNodes] = useState({});
  const [topicInfo, setTopicInfo] = useState(null);
  const [topicInfoLoading, setTopicInfoLoading] = useState(false);
  const [graphServiceData, setGraphServiceData] = useState(null);
  const [graphServiceLoading, setGraphServiceLoading] = useState(false);
  const [graphServiceError, setGraphServiceError] = useState(null);

  // Añadir referencias para D3
  const d3SimRef = useRef(null);
  const d3NodesRef = useRef([]);
  const d3LinksRef = useRef([]);
  const d3SvgRef = useRef(null);
  const d3GRef = useRef(null);

  // Al montar, lee el topic guardado
  useEffect(() => {
    console.log('useEffect called with:', panelId);
    if (!panelId) {
      console.warn('panelId is undefined, cannot load saved topic');
      return;
    }
    const key = `rosmonitor_panel_topic_${panelId}`;
    const saved = localStorage.getItem(key);
    console.log(`panelId: ${panelId}, saved: ${saved}`);
    if (saved) {
      setSelectedTopic(saved);
    }
  }, [panelId]);

  // Fetch topic info when host changes and there's a selected topic
  useEffect(() => {
    if (selectedTopic && host) {
      fetchTopicInfo(selectedTopic);
    }
  }, [host, selectedTopic]);

  // Al cambiar, guarda el topic en localStorage
  const handleTopicChange = (topic) => {
    setSelectedTopic(topic);
    const key = `rosmonitor_panel_topic_${panelId}`;
    localStorage.setItem(key, topic);
    
    // Fetch verbose topic info when topic changes
    if (topic) {
      fetchTopicInfo(topic);
    } else {
      setTopicInfo(null);
    }
  };

  // Function to fetch verbose topic information
  const fetchTopicInfo = (topicName) => {
    setTopicInfoLoading(true);
    setTopicInfo(null);
    
    const ros = rosInit(host);
    
    // Try to get verbose info using a custom ROS2 service
    const topicInfoService = new ROSLIB.Service({
      ros: ros,
      name: '/get_topic_info',
      serviceType: 'ros2_monitor_srvs/GetTopicInfo'
    });
    
    const request = new ROSLIB.ServiceRequest({
      topic_name: topicName  // Pass topic name in the topic_name field
    });
    
    topicInfoService.callService(request, (result) => {
      if (result && result.success) {
        try {
          const parsedInfo = JSON.parse(result.message);
          setTopicInfo({
            name: topicName,
            type: parsedInfo.type || 'Unknown',
            publishers: parsedInfo.publishers || [],
            subscribers: parsedInfo.subscribers || [],
            publisherCount: parsedInfo.publisher_count || 0,
            subscriberCount: parsedInfo.subscriber_count || 0,
            rawInfo: parsedInfo
          });
        } catch (error) {
          console.error('Error parsing topic info:', error);
          setTopicInfo({ error: 'Error parsing topic information' });
        }
      } else {
        console.warn('Service call failed, falling back to basic info');
        // Fallback to basic info
        fetchBasicTopicInfo(topicName, ros);
      }
      setTopicInfoLoading(false);
    }, (error) => {
      console.warn('Service not available, falling back to basic info:', error);
      // Fallback to basic info
      fetchBasicTopicInfo(topicName, ros);
    });
  };

  // Fallback function for basic topic info
  const fetchBasicTopicInfo = (topicName, ros) => {
    ros.getTopicType(topicName, (type) => {
      if (!type) {
        setTopicInfo({ error: 'Could not get topic type' });
        setTopicInfoLoading(false);
        return;
      }
      
      const topicInfo = {
        name: topicName,
        type: type,
        publishers: ['Publisher info not available via WebSocket'],
        subscribers: ['Subscriber info not available via WebSocket'],
        publisherCount: 0,
        subscriberCount: 0,
        note: 'For detailed publisher/subscriber info, use "ros2 topic info --verbose" in terminal'
      };
      
      setTopicInfo(topicInfo);
      setTopicInfoLoading(false);
    }, (error) => {
      console.error('Error getting topic type:', error);
      setTopicInfo({ error: 'Could not get topic type' });
      setTopicInfoLoading(false);
    });
  };

  // Recreate ros connection when host or selectedTopic changes
  useEffect(() => {
    const ros = rosInit(host);
    setIsConnected(false);
    ros.on('connection', () => setIsConnected(true));
    ros.on('close', () => setIsConnected(false));
    ros.on('error', () => setIsConnected(false));
    ros.getTopics(
      (result) => {
        setTopics(result.topics || []);
        setLoading(false);
      },
      (err) => {
        setError('Error fetching topics: ' + err?.toString());
        setLoading(false);
      }
    );
    return () => {
      ros.close && ros.close();
    };
  }, [host]);

  useEffect(() => {
    if (listener) {
      listener.unsubscribe();
      setListener(null);
    }
    setMessages([]);
    timestampsRef.current = [];
    setLatestMessage(null);
    if (selectedTopic) {
      const ros = rosInit(host);
      ros.getTopicType(selectedTopic, (type) => {
        if (!type) {
          console.warn(`Could not get message type for topic: ${selectedTopic}`);
        } else {
          console.log(`Subscribing to topic: ${selectedTopic} with type: ${type}`);
        }
        const newListener = new ROSLIB.Topic({
          ros,
          name: selectedTopic,
          messageType: type || 'std_msgs/String',
        });
        newListener.subscribe((message) => {
          setLatestMessage(message);
          setMessages((prev) => [message, ...prev].slice(0, 10));
          // Store timestamp for Hz calculation
          const now = Date.now();
          timestampsRef.current = [now, ...timestampsRef.current].slice(0, 10);
        });
        setListener(newListener);
      }, (err) => {
        console.error('Error getting topic type:', err);
      });
    }
    return () => {
      if (listener) listener.unsubscribe();
    };
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [selectedTopic, host]);

  // Filter topics by the filter string
  const filteredTopics = topics.filter((topic) => topic.toLowerCase().includes(filter.toLowerCase()));
  const isNodesMode = viewMode === 'nodes';

  // Calculate frequency (Hz) from timestamps
  let hz = 0;
  if (timestampsRef.current.length > 1) {
    const times = timestampsRef.current;
    const intervals = times.slice(0, -1).map((t, i) => t - times[i + 1]);
    const avgInterval = intervals.reduce((a, b) => a + b, 0) / intervals.length;
    hz = avgInterval > 0 ? (1000 / avgInterval) : 0;
  }

  // Al seleccionar un tópico, obtener el tipo de mensaje y su definición
  useEffect(() => {
    if (!selectedTopic || viewMode !== 'chart') {
      setNumericFields([]);
      setMessageType('');
      return;
    }
    const ros = rosInit(host);
    ros.getTopicType(selectedTopic, (type) => {
      setMessageType(type);
      // Llamar a /rosapi/message_details para obtener la definición
      const service = new ROSLIB.Service({
        ros,
        name: '/rosapi/message_details',
        serviceType: 'rosapi/GetMessageDetails',
      });
      const request = new ROSLIB.ServiceRequest({
        type,
      });
      service.callService(request, (result) => {
        console.log('rosapi message_details result:', result); // <-- log para depuración
        // result.typedefs es un array de definiciones
        // Buscar campos numéricos recursivamente
        const typedefs = result.typedefs || [];
        const root = typedefs.find(d => d.type === type);
        if (!root) {
          setNumericFields([]);
          return;
        }
        // Construir un mapa de typedefs para referencias
        const typedefMap = {};
        typedefs.forEach(td => { typedefMap[td.type] = td; });
        // Recursivo para encontrar campos numéricos y arrays numéricos
        function findNumericFields(fields, prefix = '') {
          let out = [];
          for (const field of fields) {
            const path = prefix ? prefix + '.' + field.name : field.name;
            const typeLower = field.type.toLowerCase();
            // Escalares numéricos
            if ([
              'float32','float64','int8','int16','int32','int64','uint8','uint16','uint32','uint64'
            ].includes(typeLower)) {
              out.push(path);
            }
            // Arrays numéricos (ej: uint8[])
            else if (typeLower.endsWith('[]')) {
              const baseType = typeLower.replace('[]','');
              if ([
                'float32','float64','int8','int16','int32','int64','uint8','uint16','uint32','uint64'
              ].includes(baseType)) {
                out.push(path + '[0]'); // graficar primer elemento
              }
            }
            // Submensajes
            else if (typedefMap[field.type]) {
              out = out.concat(findNumericFields(
                typedefMap[field.type].fieldnames.map((name, i) => ({ name, type: typedefMap[field.type].fieldtypes[i] })),
                path
              ));
            }
          }
          return out;
        }
        const fields = findNumericFields(root.fieldnames.map((name, i) => ({ name, type: root.fieldtypes[i] })));
        setNumericFields(fields);
        // Solo restaurar si el topic cambió
        if (prevTopicRef.current !== selectedTopic) {
          const map = JSON.parse(localStorage.getItem('rosbridge_field_map') || '{}');
          if (map[panelId] && map[panelId][selectedTopic] && fields.includes(map[panelId][selectedTopic])) {
            setSelectedField(map[panelId][selectedTopic]);
          } else {
            setSelectedField(fields[0] || '');
          }
          prevTopicRef.current = selectedTopic;
        }
      });
    });
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [selectedTopic, host, viewMode, panelId]);

  // Chart rendering effect
  chartEffect(() => {
    if (viewMode !== 'chart' || !chartCanvasRef.current || !selectedField) return;
    // Extract values for the chart
    const getValue = (msg, path) => {
      // Soporta path tipo 'data[0]' para arrays
      const parts = path.split('.');
      let val = msg;
      for (const p of parts) {
        const arrMatch = p.match(/(\w+)\[(\d+)\]/);
        if (arrMatch) {
          val = val?.[arrMatch[1]];
          val = Array.isArray(val) ? val[parseInt(arrMatch[2], 10)] : undefined;
        } else {
          val = val?.[p];
        }
      }
      return typeof val === 'number' ? val : null;
    };
    const data = messages.slice().reverse().map(m => getValue(m, selectedField));
    const labels = messages.slice().reverse().map((_, i) => i + 1);
    if (chartInstanceRef.current) chartInstanceRef.current.destroy();
    chartInstanceRef.current = new Chart(chartCanvasRef.current, {
      type: 'line',
      data: {
        labels,
        datasets: [{
          label: selectedField,
          data,
          borderColor: '#646cff',
          backgroundColor: 'rgba(100,108,255,0.1)',
          tension: 0.2,
        }],
      },
      options: {
        animation: {
          duration: 200, // faster animation
        },
        scales: {
          x: {
            title: { display: true, text: 'Message count' },
            ticks: { color: '#ddd' },
            grid: { color: '#444' }
          },
          y: {
            title: { display: true, text: 'Value' },
            ticks: { color: '#ddd' },
            grid: { color: '#444' }
          }
        },
        plugins: {
          legend: {
            labels: { color: '#ddd' }
          },
          title: {
            display: true,
            text: `${selectedTopic} - ${selectedField}`,
            color: '#ddd',
            font: { size: 16 }
          }
        }
      },
    });
    return () => {
      if (chartInstanceRef.current) chartInstanceRef.current.destroy();
    };
  }, [messages, selectedField, viewMode, chartCanvasRef, selectedTopic]);

  // Fetch nodes and topics when 'nodes' view is selected
  useEffect(() => {
    if (viewMode !== 'nodes') return;
    const ros = rosInit(host);
    // Fetch nodes
    const nodesService = new ROSLIB.Service({
      ros,
      name: '/rosapi/nodes',
      serviceType: 'rosapi/Nodes',
    });
    nodesService.callService(new ROSLIB.ServiceRequest({}), (result) => {
      setNodes(result.nodes || []);
    });
    // Fetch topics
    const topicsService = new ROSLIB.Service({
      ros,
      name: '/rosapi/topics',
      serviceType: 'rosapi/Topics',
    });
    topicsService.callService(new ROSLIB.ServiceRequest({}), (result) => {
      setNodeTopics(result.topics || []);
    });
    // Fetch publishers
    const publishersService = new ROSLIB.Service({
      ros,
      name: '/rosapi/publishers',
      serviceType: 'rosapi/Publishers',
    });
    publishersService.callService(new ROSLIB.ServiceRequest({}), (result) => {
      setPublishers(result.publishers || {});
    });
    // Fetch subscribers
    const subscribersService = new ROSLIB.Service({
      ros,
      name: '/rosapi/subscribers',
      serviceType: 'rosapi/Subscribers',
    });
    subscribersService.callService(new ROSLIB.ServiceRequest({}), (result) => {
      setSubscribers(result.subscribers || {});
    });
    // Fetch services
    const servicesService = new ROSLIB.Service({
      ros,
      name: '/rosapi/services',
      serviceType: 'rosapi/Services',
    });
    servicesService.callService(new ROSLIB.ServiceRequest({}), (result) => {
      setServices(result.services || []);
    });
    // Fetch service_node (which node offers each service)
    // We'll fetch for each service
    setServiceNodes({}); // clear before fetching
    (result => {
      if (!result.services) return;
      const serviceNodeMap = {};
      let count = 0;
      result.services.forEach(service => {
        const serviceNodeService = new ROSLIB.Service({
          ros,
          name: '/rosapi/service_node',
          serviceType: 'rosapi/ServiceNode',
        });
        serviceNodeService.callService(new ROSLIB.ServiceRequest({ service }), (res) => {
          serviceNodeMap[service] = res.node;
          count++;
          if (count === result.services.length) {
            setServiceNodes(serviceNodeMap);
          }
        });
      });
    })({ services: [] }); // will be called again below after services fetched
    // When services are fetched, fetch service_node for each
    servicesService.callService(new ROSLIB.ServiceRequest({}), (result) => {
      if (!result.services) return;
      const serviceNodeMap = {};
      let count = 0;
      result.services.forEach(service => {
        const serviceNodeService = new ROSLIB.Service({
          ros,
          name: '/rosapi/service_node',
          serviceType: 'rosapi/ServiceNode',
        });
        serviceNodeService.callService(new ROSLIB.ServiceRequest({ service }), (res) => {
          serviceNodeMap[service] = res.node;
          count++;
          if (count === result.services.length) {
            setServiceNodes(serviceNodeMap);
          }
        });
      });
    });
  }, [viewMode, host]);

  // Build graph data
  const allNodes = [
    ...nodes.map(id => ({ id, type: 'node' })),
    ...nodeTopics.map(id => ({ id, type: 'topic' })),
    ...services.map(id => ({ id, type: 'service' })),
  ];
  const nodeMap = Object.fromEntries(allNodes.map(n => [n.id, n]));
  let links = [];
  // Topic publishers: node --(publish)--> topic
  Object.entries(publishers).forEach(([topic, pubs]) => {
    pubs.forEach(node => {
      if (nodeMap[node] && nodeMap[topic]) {
        links.push({ source: node, target: topic, type: 'publish' });
      }
    });
  });
  // Topic subscribers: topic --(subscribe)--> node
  Object.entries(subscribers).forEach(([topic, subs]) => {
    subs.forEach(node => {
      if (nodeMap[node] && nodeMap[topic]) {
        links.push({ source: topic, target: node, type: 'subscribe' });
      }
    });
  });
  // Services: node --(offers)--> service
  Object.entries(serviceNodes).forEach(([service, node]) => {
    if (nodeMap[node] && nodeMap[service]) {
      links.push({ source: node, target: service, type: 'service' });
    }
  });
  // Filter out orphan nodes (no links)
  const connectedIds = new Set();
  links.forEach(l => { connectedIds.add(l.source); connectedIds.add(l.target); });
  const nodeList = allNodes.filter(n => connectedIds.has(n.id));

  // Llama automáticamente al servicio al entrar en la vista 'nodes'
  useEffect(() => {
    if (viewMode !== 'nodes') return;
    fetchGraphInfo();
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [viewMode, host]);

  // Reemplaza el efecto D3.js para el grafo de nodos para usar graphServiceData
  useEffect(() => {
    if (viewMode !== 'nodes' || !graphServiceData) return;
    const containerId = `d3-nodes-graph-${panelId}`;
    const container = document.getElementById(containerId);
    if (!container) return;

    // Build nodes and links from graphServiceData
    let allNodes = [];
    let links = [];
    const TOPIC_FILTER = ['/rosout', '/parameter_events'];
    const isFiltered = name => TOPIC_FILTER.includes(name);
    allNodes = [
      ...(graphServiceData.nodes || []).map(n => ({ id: n.name, type: 'node' })),
      ...(graphServiceData.topics || []).filter(t => !isFiltered(t.name)).map(t => ({ id: t.name, type: 'topic' })),
      ...(graphServiceData.services || []).filter(s => !isFiltered(s.name)).map(s => ({ id: s.name, type: 'service' })),
    ];
    (graphServiceData.publishers || []).forEach(pub => {
      if (!isFiltered(pub.topic))
        links.push({ source: pub.node, target: pub.topic, type: 'publish' });
    });
    (graphServiceData.subscribers || []).forEach(sub => {
      if (!isFiltered(sub.topic))
        links.push({ source: sub.topic, target: sub.node, type: 'subscribe' });
    });
    if (graphServiceData.service_providers) {
      graphServiceData.service_providers.forEach(sp => {
        if (!isFiltered(sp.service))
          links.push({ source: sp.node, target: sp.service, type: 'service' });
      });
    }
    // Identificar nodos huérfanos
    const connectedIds = new Set();
    links.forEach(l => { connectedIds.add(l.source); connectedIds.add(l.target); });
    const nodeList = allNodes;
    const width = container.offsetWidth || 600;
    const height = container.offsetHeight || 400;

    // --- SVG y G persistentes ---
    let svg = d3SvgRef.current;
    let g = d3GRef.current;
    if (!svg) {
      svg = d3.select(container)
        .append('svg')
        .attr('width', width)
        .attr('height', height)
        .style('background', '#222');
      d3SvgRef.current = svg;
      g = svg.append('g');
      d3GRef.current = g;
      svg.call(d3.zoom()
        .scaleExtent([0.1, 2])
        .on('zoom', (event) => {
          g.attr('transform', event.transform);
        })
      );
    }

    // --- Mantener posiciones previas ---
    const prevNodes = d3NodesRef.current;
    const prevNodeMap = Object.fromEntries(prevNodes.map(n => [n.id, n]));
    nodeList.forEach(n => {
      const prev = prevNodeMap[n.id];
      if (prev) {
        n.x = prev.x;
        n.y = prev.y;
        n.vx = prev.vx;
        n.vy = prev.vy;
        n.fx = prev.fx;
        n.fy = prev.fy;
      }
    });
    d3NodesRef.current = nodeList;
    d3LinksRef.current = links;

    // --- Data join para links ---
    // Asegura que source y target sean siempre ids (string)
    links = links.map(l => ({
      ...l,
      source: typeof l.source === 'object' ? l.source.id : l.source,
      target: typeof l.target === 'object' ? l.target.id : l.target
    }));
    const linkSel = g.selectAll('line').data(links, d => d.source + '-' + d.target + '-' + d.type);
    linkSel.exit().remove();
    linkSel.enter()
      .append('line')
      .attr('stroke', '#ffd93d')
      .attr('stroke-width', 10)
      .attr('stroke-dasharray', d => d.type === 'service' ? '4,2' : '');
    // --- Data join para nodos ---
    const nodeSel = g.selectAll('circle').data(nodeList, d => d.id);
    nodeSel.exit().remove();
    nodeSel.enter()
      .append('circle')
      .attr('stroke', '#fff')
      .attr('stroke-width', 2)
      .attr('r', d => d.type === 'node' ? 44 : d.type === 'topic' ? 32 : 24)
      .attr('fill', d => d.type === 'node' ? '#2ecc40' : d.type === 'topic' ? '#43e' : '#fa0')
      .call(d3.drag()
        .on('start', dragstarted)
        .on('drag', dragged)
        .on('end', dragended));
    // --- Data join para labels ---
    const labelSel = g.selectAll('text').data(nodeList, d => d.id);
    labelSel.exit().remove();
    labelSel.enter()
      .append('text')
      .attr('fill', '#fff')
      .attr('font-size', 14)
      .attr('text-anchor', 'middle')
      .attr('dy', 5)
      .text(d => d.id);
    // --- Simulación de fuerzas ---
    if (!d3SimRef.current) {
      d3SimRef.current = d3.forceSimulation(nodeList)
        .force('link', d3.forceLink(links).id(d => d.id).distance(120))
        .force('charge', d3.forceManyBody().strength(-400))
        .force('center', d3.forceCenter(width / 2, height / 2))
        .on('tick', ticked);
    } else {
      d3SimRef.current.nodes(nodeList);
      d3SimRef.current.force('link').links(links);
      d3SimRef.current.alpha(1).restart();
    }
    function ticked() {
      // Log de posiciones de nodos
      console.log('[DEBUG][D3] Posiciones de nodos:', d3NodesRef.current.map(n => ({ id: n.id, x: n.x, y: n.y })));
      // Log de posiciones de links
      links.forEach(l => {
        const n1 = typeof l.source === 'object' ? l.source : d3NodesRef.current.find(n => n.id === l.source);
        const n2 = typeof l.target === 'object' ? l.target : d3NodesRef.current.find(n => n.id === l.target);
        console.log(`[DEBUG][D3] Link ${n1?.id} -> ${n2?.id}: x1=${n1?.x}, y1=${n1?.y}, x2=${n2?.x}, y2=${n2?.y}`);
      });
      g.selectAll('line')
        .attr('x1', d => (typeof d.source === 'object' ? d.source.x : d3NodesRef.current.find(n => n.id === d.source)?.x))
        .attr('y1', d => (typeof d.source === 'object' ? d.source.y : d3NodesRef.current.find(n => n.id === d.source)?.y))
        .attr('x2', d => (typeof d.target === 'object' ? d.target.x : d3NodesRef.current.find(n => n.id === d.target)?.x))
        .attr('y2', d => (typeof d.target === 'object' ? d.target.y : d3NodesRef.current.find(n => n.id === d.target)?.y));
      g.selectAll('circle')
        .attr('cx', d => d.x)
        .attr('cy', d => d.y);
      g.selectAll('text')
        .attr('x', d => d.x)
        .attr('y', d => d.y);
    }
    function dragstarted(event, d) {
      if (!event.active && d3SimRef.current) d3SimRef.current.alphaTarget(0.3).restart();
      d.fx = d.x;
      d.fy = d.y;
    }
    function dragged(event, d) {
      d.fx = event.x;
      d.fy = event.y;
    }
    function dragended(event, d) {
      if (!event.active && d3SimRef.current) d3SimRef.current.alphaTarget(0);
      d.fx = null;
      d.fy = null;
    }
    // Cleanup on unmount
    return () => {
      if (d3SimRef.current) d3SimRef.current.stop();
      d3SimRef.current = null;
      if (svg) svg.remove();
      d3SvgRef.current = null;
      d3GRef.current = null;
    };
  }, [viewMode, graphServiceData, panelId]);

  // Función para llamar al servicio get_graph_info
  const fetchGraphInfo = () => {
    setGraphServiceLoading(true);
    setGraphServiceError(null);
    setGraphServiceData(null);
    const ros = rosInit(host);
    const graphService = new ROSLIB.Service({
      ros: ros,
      name: '/get_graph_info',
      serviceType: 'ros2_monitor_srvs/GetGraphInfo'
    });
    const request = new ROSLIB.ServiceRequest({});
    graphService.callService(request, (result) => {
      try {
        const data = JSON.parse(result.graph_json);
        setGraphServiceData(data);
        console.log('[DEBUG] Respuesta completa de get_graph_info:', result);
        if (result && result.graph_json) {
          try {
            const graph = typeof result.graph_json === 'string' ? JSON.parse(result.graph_json) : result.graph_json;
            console.log('[DEBUG] graph.topics:', graph.topics);
            console.log('[DEBUG] graph.nodes:', graph.nodes);
            console.log('[DEBUG] graph.publishers:', graph.publishers);
            console.log('[DEBUG] graph.subscribers:', graph.subscribers);
            // Construye el array de nodos para D3
            const nodes = [
              ...(graph.nodes ? graph.nodes.map(n => ({ id: n.name, ...n })) : []),
              ...(graph.topics ? graph.topics.map(t => ({ id: t.name, ...t })) : [])
            ];
            console.log('[DEBUG] nodos para D3:', nodes);
            // Construye los links
            const links = [
              ...(graph.publishers ? graph.publishers.map(pub => ({ source: pub.node, target: pub.topic, type: 'pub' })) : []),
              ...(graph.subscribers ? graph.subscribers.map(sub => ({ source: sub.node, target: sub.topic, type: 'sub' })) : [])
            ];
            // Chequea que todos los source/target existen en nodos
            const nodeIds = new Set(nodes.map(n => n.id));
            const missingSources = links.filter(l => !nodeIds.has(l.source));
            const missingTargets = links.filter(l => !nodeIds.has(l.target));
            if (missingSources.length > 0) {
              console.warn('[DEBUG] Links con source que NO existe en nodos:', missingSources);
            }
            if (missingTargets.length > 0) {
              console.warn('[DEBUG] Links con target que NO existe en nodos:', missingTargets);
            }
          } catch (e) {
            console.error('[DEBUG] Error parseando graph_json:', e);
          }
        }
      } catch (e) {
        setGraphServiceError('Error parsing graph_json: ' + e);
      }
      setGraphServiceLoading(false);
    }, (err) => {
      setGraphServiceError('Service call failed: ' + (err?.toString() || 'unknown error'));
      setGraphServiceLoading(false);
    });
  };

  return (
    <div className="ros-monitor-widget">
      <div className="widget-header">
        <span style={{ fontSize: 13, color: '#888', marginRight: 8 }}>ID: {panelId}</span>
        <a href="#" onClick={() => window.location.reload()} style={{ marginRight: '1em' }}>
          <img src={githubLogo} alt="GitHub Logo" style={{ width: '32px', height: '32px' }}/>
        </a>
      </div>

      {error && <p className="error-message">{error}</p>}

      {/* Only show topic selection and info if not in nodes mode */}
      {!isNodesMode && (
        <>
          <div className="topic-selection">
            <input
              type="text"
              placeholder="Filter topics..."
              value={filter}
              onChange={(e) => setFilter(e.target.value)}
              style={{ marginBottom: '0.5em', width: 'calc(100% - 12px)' }}
            />
            {loading ? (
              <p>Loading topics...</p>
            ) : (
              <select
                onChange={e => handleTopicChange(e.target.value)}
                value={selectedTopic}
                size={5}
              >
                <option value="">-- Select a topic --</option>
                {filteredTopics.map((topic) => (
                  <option key={topic} value={topic}>
                    {topic}
                  </option>
                ))}
              </select>
            )}
          </div>

          {selectedTopic && (
            <div className="topic-info">
              <h3>
                Topic: {selectedTopic}{' '}
                <span style={{ color: '#aaa', fontWeight: 'normal' }}>
                  ({hz.toFixed(2)} Hz)
                </span>
              </h3>

              {/* Verbose Topic Information */}
              <div className="verbose-topic-info" style={{ 
                background: '#2a2a2a', 
                border: '1px solid #444', 
                borderRadius: '4px', 
                padding: '12px', 
                marginBottom: '16px',
                fontSize: '12px'
              }}>
                <div style={{ display: 'flex', justifyContent: 'space-between', alignItems: 'center', marginBottom: '8px' }}>
                  <h4 style={{ margin: 0, color: '#fff' }}>Topic Information (Verbose)</h4>
                  <button 
                    onClick={() => selectedTopic && fetchTopicInfo(selectedTopic)}
                    disabled={topicInfoLoading}
                    style={{
                      background: '#4ecdc4',
                      color: '#000',
                      border: 'none',
                      borderRadius: '4px',
                      padding: '4px 8px',
                      fontSize: '10px',
                      cursor: topicInfoLoading ? 'not-allowed' : 'pointer',
                      opacity: topicInfoLoading ? 0.6 : 1
                    }}
                  >
                    {topicInfoLoading ? 'Loading...' : 'Refresh'}
                  </button>
                </div>
                {topicInfoLoading ? (
                  <p style={{ color: '#888', margin: 0 }}>Loading topic information...</p>
                ) : topicInfo?.error ? (
                  <p style={{ color: '#ff6b6b', margin: 0 }}>Error: {topicInfo.error}</p>
                ) : topicInfo ? (
                  <div style={{ color: '#ccc' }}>
                    <div style={{ marginBottom: '8px' }}>
                      <strong>Type:</strong> <span style={{ color: '#4ecdc4' }}>{topicInfo.type}</span>
                    </div>
                    <div style={{ marginBottom: '8px' }}>
                      <details>
                        <summary style={{fontWeight: 'bold', fontSize: 15, cursor: 'pointer', color: '#4ecdc4', padding: '4px 0'}}>
                          Publishers ({topicInfo.publisherCount || topicInfo.publishers.length})
                        </summary>
                        <ul style={{ margin: '4px 0', paddingLeft: '20px' }}>
                          {topicInfo.publishers.map((pub, index) => (
                            <li key={index} style={{ color: '#4ecdc4', marginBottom: 8, background: '#23272f', borderRadius: 4, padding: 0 }}>
                              <details>
                                <summary style={{padding: '6px 10px', cursor: 'pointer'}}>
                                  <strong>Node:</strong> {pub.node_name} <span style={{ color: '#ffd93d', marginLeft: 8 }}>[{pub.node_namespace}]</span>
                                </summary>
                                <div style={{padding: '6px 10px'}}>
                                  <div><strong>GID:</strong> <span style={{ color: '#aaa', fontSize: 11 }}>{pub.gid}</span></div>
                                  {pub.qos && Object.keys(pub.qos).length > 0 && (
                                    <div style={{ marginTop: 4 }}>
                                      <strong>QoS:</strong>
                                      <ul style={{ margin: '2px 0 2px 16px', padding: 0, listStyle: 'circle', color: '#ffd93d', fontSize: 12 }}>
                                        {Object.entries(pub.qos).map(([k, v]) => (
                                          <li key={k}><strong>{k}:</strong> {v}</li>
                                        ))}
                                      </ul>
                                    </div>
                                  )}
                                </div>
                              </details>
                            </li>
                          ))}
                        </ul>
                      </details>
                    </div>
                    <div style={{ marginBottom: '8px' }}>
                      <details>
                        <summary style={{fontWeight: 'bold', fontSize: 15, cursor: 'pointer', color: '#ffd93d', padding: '4px 0'}}>
                          Subscribers ({topicInfo.subscriberCount || topicInfo.subscribers.length})
                        </summary>
                        <ul style={{ margin: '4px 0', paddingLeft: '20px' }}>
                          {topicInfo.subscribers.map((sub, index) => (
                            <li key={index} style={{ color: '#ffd93d', marginBottom: 8, background: '#23272f', borderRadius: 4, padding: 0 }}>
                              <details>
                                <summary style={{padding: '6px 10px', cursor: 'pointer'}}>
                                  <strong>Node:</strong> {sub.node_name} <span style={{ color: '#4ecdc4', marginLeft: 8 }}>[{sub.node_namespace}]</span>
                                </summary>
                                <div style={{padding: '6px 10px'}}>
                                  <div><strong>GID:</strong> <span style={{ color: '#aaa', fontSize: 11 }}>{sub.gid}</span></div>
                                  {sub.qos && Object.keys(sub.qos).length > 0 && (
                                    <div style={{ marginTop: 4 }}>
                                      <strong>QoS:</strong>
                                      <ul style={{ margin: '2px 0 2px 16px', padding: 0, listStyle: 'circle', color: '#6bcf7f', fontSize: 12 }}>
                                        {Object.entries(sub.qos).map(([k, v]) => (
                                          <li key={k}><strong>{k}:</strong> {v}</li>
                                        ))}
                                      </ul>
                                    </div>
                                  )}
                                </div>
                              </details>
                            </li>
                          ))}
                        </ul>
                      </details>
                    </div>
                    {/* QoS global info (if any) */}
                    {topicInfo.qos && topicInfo.qos.length > 0 && (
                      <div style={{ marginBottom: '8px' }}>
                        <strong>QoS Profiles:</strong>
                        <ul style={{ margin: '4px 0', paddingLeft: '20px', color: '#ffd93d' }}>
                          {topicInfo.qos.map((q, i) => (
                            <li key={i}>
                              {Object.entries(q).map(([k, v]) => (
                                <span key={k} style={{ marginRight: 8 }}><strong>{k}:</strong> {v}</span>
                              ))}
                            </li>
                          ))}
                        </ul>
                      </div>
                    )}
                    {/* Extended info for debug */}
                    {topicInfo.publishers_info && topicInfo.publishers_info.length > 0 && (
                      <details style={{ marginBottom: '8px' }}>
                        <summary>Raw Publishers Info</summary>
                        <pre style={{ background: '#222', color: '#fff', padding: 8, borderRadius: 4, fontSize: 11 }}>{topicInfo.publishers_info.join('\n')}</pre>
                      </details>
                    )}
                    {topicInfo.subscribers_info && topicInfo.subscribers_info.length > 0 && (
                      <details style={{ marginBottom: '8px' }}>
                        <summary>Raw Subscribers Info</summary>
                        <pre style={{ background: '#222', color: '#fff', padding: 8, borderRadius: 4, fontSize: 11 }}>{topicInfo.subscribers_info.join('\n')}</pre>
                      </details>
                    )}
                    {topicInfo.messageCount > 0 && (
                      <div style={{ marginBottom: '8px' }}>
                        <strong>Message Count:</strong> <span style={{ color: '#ff6b6b' }}>{topicInfo.messageCount}</span>
                      </div>
                    )}
                    {topicInfo.frequency > 0 && (
                      <div style={{ marginBottom: '8px' }}>
                        <strong>Frequency:</strong> <span style={{ color: '#6bcf7f' }}>{topicInfo.frequency.toFixed(2)} Hz</span>
                      </div>
                    )}
                    {topicInfo.note && (
                      <div style={{ 
                        marginTop: '8px', 
                        padding: '6px', 
                        background: '#3a3a3a', 
                        borderRadius: '3px', 
                        borderLeft: '3px solid #ffd93d',
                        fontSize: '11px'
                      }}>
                        <strong>Note:</strong> {topicInfo.note}
                      </div>
                    )}
                  </div>
                ) : (
                  <p style={{ color: '#888', margin: 0 }}>No topic information available</p>
                )}
              </div>

              {/* View toggle only if not in nodes mode */}
              <div className="view-toggle">
                <button onClick={() => setViewMode('messages')} className={viewMode === 'messages' ? 'active' : ''}>Messages</button>
                <button onClick={() => setViewMode('chart')} className={viewMode === 'chart' ? 'active' : ''}>Chart</button>
              </div>

              {viewMode === 'messages' && (
                <div className="messages-view">
                  <div style={{ marginBottom: '1em' }}>
                    <label>
                      <input type="checkbox" checked={showHex} onChange={(e) => setShowHex(e.target.checked)} />
                      Show HEX
                    </label>
                  </div>
                  <h4>Latest Message:</h4>
                  {latestMessage ? (
                    <MessageRenderer data={latestMessage} showHex={showHex} />
                  ) : (
                    <p>Waiting for the first message...</p>
                  )}
                </div>
              )}

              {viewMode === 'chart' && (
                <div className="chart-view">
                  {numericFields.length > 0 ? (
                    <>
                      <select onChange={(e) => setSelectedField(e.target.value)} value={selectedField}>
                        <option value="">-- Select a field to plot --</option>
                        {numericFields.map(f => <option key={f} value={f}>{f}</option>)}
                      </select>
                      <div className="chart-container">
                        <canvas ref={chartCanvasRef}></canvas>
                      </div>
                    </>
                  ) : (
                    <p>No plottable (numeric) fields found in message type: {messageType}</p>
                  )}
                </div>
              )}
            </div>
          )}
        </>
      )}

      {/* Only show nodes view if in nodes mode */}
      {isNodesMode && (
        <div className="nodes-view">
          {/* Botón para llamar al servicio get_graph_info */}
          <div style={{ marginBottom: 12 }}>
            <button onClick={fetchGraphInfo} disabled={graphServiceLoading} style={{ background: '#4ecdc4', color: '#000', border: 'none', borderRadius: 4, padding: '4px 12px', fontSize: 13, cursor: graphServiceLoading ? 'not-allowed' : 'pointer', opacity: graphServiceLoading ? 0.6 : 1 }}>
              {graphServiceLoading ? 'Loading graph info...' : 'Fetch Graph Info (Service)'}
            </button>
          </div>
          {/* Panel para mostrar el resultado del servicio */}
          {graphServiceError && <div style={{ color: '#ff6b6b', marginBottom: 8 }}>Error: {graphServiceError}</div>}
          {/* D3 graph container with fixed size and border for debug */}
          <div id={`d3-nodes-graph-${panelId}`} style={{ width: 800, maxWidth: '100%', height: 500, margin: '24px auto', background: '#222', border: '2px solid #646cff', borderRadius: 8, color: '#fff', display: 'flex', alignItems: 'center', justifyContent: 'center' }}>
            {/* D.js node graph will appear here. */}
          </div>
        </div>
      )}
    </div>
  );
}

export default RosMonitorWidget; 