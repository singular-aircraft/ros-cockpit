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

function RosMonitorWidget({ panelId, host }) {
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
  const [viewMode, setViewMode] = useState('messages'); // 'messages', 'chart', or 'nodes'
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
  const [graphData, setGraphData] = useState(null);

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
    if (saved) setSelectedTopic(saved);
  }, [panelId]);

  // Al cambiar, guarda el topic en localStorage
  const handleTopicChange = (topic) => {
    setSelectedTopic(topic);
    const key = `rosmonitor_panel_topic_${panelId}`;
    localStorage.setItem(key, topic);
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

  // Debug: log publishers and subscribers
  console.log('publishers:', publishers);
  console.log('subscribers:', subscribers);

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

  // Subscribe to /ros2_graph topic if available
  useEffect(() => {
    if (viewMode !== 'nodes') return;
    const ros = rosInit(host);
    const topic = new ROSLIB.Topic({
      ros,
      name: '/ros2_graph',
      messageType: 'std_msgs/String',
    });
    const handler = (msg) => {
      try {
        const data = JSON.parse(msg.data);
        setGraphData(data);
      } catch (e) {
        console.warn('Failed to parse /ros2_graph message:', e);
      }
    };
    topic.subscribe(handler);
    return () => {
      topic.unsubscribe();
    };
  }, [viewMode, host]);

  // D3.js effect for node graph (use graphData if available)
  useEffect(() => {
    if (viewMode !== 'nodes') return;
    const containerId = `d3-nodes-graph-${panelId}`;
    const container = document.getElementById(containerId);
    if (!container) return;
    container.innerHTML = '';

    // Use graphData from /ros2_graph if available
    let allNodes = [];
    let links = [];
    if (graphData) {
      // Build nodes: type 'node', 'topic', 'service'
      allNodes = [
        ...(graphData.nodes || []).map(n => ({ id: n.name, type: 'node' })),
        ...(graphData.topics || []).map(t => ({ id: t.name, type: 'topic' })),
        ...(graphData.services || []).map(s => ({ id: s.name, type: 'service' })),
      ];
      // Build links from publishers/subscribers
      (graphData.publishers || []).forEach(pub => {
        links.push({ source: pub.node, target: pub.topic, type: 'publish' });
      });
      (graphData.subscribers || []).forEach(sub => {
        links.push({ source: sub.topic, target: sub.node, type: 'subscribe' });
      });
      // (Optionally, add service links if present)
      if (graphData.service_providers) {
        graphData.service_providers.forEach(sp => {
          links.push({ source: sp.node, target: sp.service, type: 'service' });
        });
      }
    } else {
      // Fallback to old logic (no links in ROS2)
      // Filter out parameter-related and lifecycle-related topics/services (partial match)
      const PARAM_FILTER = [
        'get_parameters', 'set_parameters', 'describe_parameters',
        'list_parameters', 'get_parameter_types', 'set_parameters_atomically',
        'available_transitions', 'change_state', 'set_state', 'get_state',
        'get_available_states', 'get_available_transitions', 'trigger_transition',
        'get_transition_graph', 'transition_event'
      ];
      const isParamRelated = name => PARAM_FILTER.some(f => name.includes(f));
      const filteredTopics = nodeTopics.filter(t => !isParamRelated(t));
      const filteredServices = services.filter(s => !isParamRelated(s));
      allNodes = [
        ...nodes.map(id => ({ id, type: 'node' })),
        ...filteredTopics.map(id => ({ id, type: 'topic' })),
        ...filteredServices.map(id => ({ id, type: 'service' })),
      ];
      // No links in fallback for ROS2
      links = [];
    }
    // Identify orphan nodes (no links)
    const connectedIds = new Set();
    links.forEach(l => { connectedIds.add(l.source); connectedIds.add(l.target); });
    const orphanNodes = allNodes.filter(n => !connectedIds.has(n.id));
    const nodeList = allNodes;
    // Build graph data
    console.log('D3 nodeList:', nodeList);
    console.log('D3 links:', links);
    // D3 force-directed graph
    const width = container.offsetWidth || 600;
    const height = container.offsetHeight || 400;
    const svg = d3.select(container)
      .append('svg')
      .attr('width', width)
      .attr('height', height)
      .style('background', '#222'); // visible SVG background

    // Add zoom/pan support
    const g = svg.append('g');
    svg.call(d3.zoom()
      .scaleExtent([0.1, 2])
      .on('zoom', (event) => {
        g.attr('transform', event.transform);
      })
    );

    // D3 simulation with extra force for orphans
    const simulation = d3.forceSimulation(nodeList)
      .force('link', d3.forceLink(links).id(d => d.id).distance(120))
      .force('charge', d3.forceManyBody().strength(-400))
      .force('center', d3.forceCenter(width / 2, height / 2))
      .force('orphanCenter', d3.forceManyBody().strength(0)) // placeholder, will override below
      .force('orphanCollide', d3.forceCollide().radius(d => orphanNodes.includes(d) ? 56 : 0).strength(1));
    // Apply strong centering force only to orphans
    simulation.force('orphanCenter', function(alpha) {
      orphanNodes.forEach(d => {
        d.vx = (width / 2 - d.x) * 0.2 * alpha;
        d.vy = (height / 2 - d.y) * 0.2 * alpha;
      });
    });

    const link = g.append('g')
      .attr('stroke', '#fff')
      .attr('stroke-width', 6)
      .selectAll('line')
      .data(links)
      .enter().append('line')
      .attr('stroke', '#fff')
      .attr('stroke-dasharray', d => d.type === 'service' ? '4,2' : '');
    const node = g.append('g')
      .attr('stroke', '#fff')
      .attr('stroke-width', 2)
      .selectAll('circle')
      .data(nodeList)
      .enter().append('circle')
      .attr('r', d => d.type === 'node' ? 44 : d.type === 'topic' ? 32 : 24)
      .attr('fill', d => d.type === 'node' ? '#2ecc40' : d.type === 'topic' ? '#43e' : '#fa0')
      .call(d3.drag()
        .on('start', dragstarted)
        .on('drag', dragged)
        .on('end', dragended));
    const label = g.append('g')
      .selectAll('text')
      .data(nodeList)
      .enter().append('text')
      .attr('text-anchor', 'middle')
      .attr('dy', 5)
      .attr('fill', '#fff')
      .attr('font-size', 9)
      .text(d => d.id.replace(/^\//, ''));
    simulation.on('tick', () => {
      // Helper to get radius by type
      function getRadius(d) {
        return d.type === 'node' ? 44 : d.type === 'topic' ? 32 : 24;
      }
      // Custom link rendering to stop at edge of circles
      link
        .attr('x1', d => {
          const r = getRadius(d.source);
          const dx = d.target.x - d.source.x;
          const dy = d.target.y - d.source.y;
          const dist = Math.sqrt(dx*dx + dy*dy) || 1;
          return d.source.x + (dx * r) / dist;
        })
        .attr('y1', d => {
          const r = getRadius(d.source);
          const dx = d.target.x - d.source.x;
          const dy = d.target.y - d.source.y;
          const dist = Math.sqrt(dx*dx + dy*dy) || 1;
          return d.source.y + (dy * r) / dist;
        })
        .attr('x2', d => {
          const r = getRadius(d.target);
          const dx = d.source.x - d.target.x;
          const dy = d.source.y - d.target.y;
          const dist = Math.sqrt(dx*dx + dy*dy) || 1;
          return d.target.x + (dx * r) / dist;
        })
        .attr('y2', d => {
          const r = getRadius(d.target);
          const dx = d.source.x - d.target.x;
          const dy = d.source.y - d.target.y;
          const dist = Math.sqrt(dx*dx + dy*dy) || 1;
          return d.target.y + (dy * r) / dist;
        });
      node
        .attr('cx', d => d.x)
        .attr('cy', d => d.y);
      label
        .attr('x', d => d.x)
        .attr('y', d => d.y);
    });

    // Drag functions for D3 nodes
    function dragstarted(event, d) {
      if (!event.active) simulation.alphaTarget(0.3).restart();
      d.fx = d.x;
      d.fy = d.y;
    }
    function dragged(event, d) {
      d.fx = event.x;
      d.fy = event.y;
    }
    function dragended(event, d) {
      if (!event.active) simulation.alphaTarget(0);
      d.fx = null;
      d.fy = null;
    }

    // Fit to view after simulation stabilizes
    function fitToView() {
      const all = nodeList;
      if (!all.length) return;
      // Validate coordinates
      if (all.some(d => typeof d.x !== 'number' || typeof d.y !== 'number' || isNaN(d.x) || isNaN(d.y))) {
        console.warn('Some node coordinates are invalid, skipping fitToView');
        return;
      }
      const minX = Math.min(...all.map(d => d.x));
      const maxX = Math.max(...all.map(d => d.x));
      const minY = Math.min(...all.map(d => d.y));
      const maxY = Math.max(...all.map(d => d.y));
      const graphWidth = maxX - minX;
      const graphHeight = maxY - minY;
      const margin = 40;
      const scale = Math.min(
        width / (graphWidth + 2 * margin),
        height / (graphHeight + 2 * margin),
        1
      );
      const tx = (width - scale * (minX + maxX)) / 2;
      const ty = (height - scale * (minY + maxY)) / 2;
      svg.transition().duration(500).call(
        d3.zoom().transform,
        d3.zoomIdentity.translate(tx, ty).scale(scale)
      );
    }
    // Wait for simulation to stabilize, then fit
    simulation.on('end', fitToView);
    // Also fit after a short timeout in case 'end' doesn't fire
    setTimeout(fitToView, 1500);

    // Cleanup on unmount or data change
    return () => {
      simulation.stop();
      container.innerHTML = '';
    };
  }, [viewMode, nodes, nodeTopics, publishers, subscribers, services, serviceNodes, panelId, graphData]);

  return (
    <div className="ros-monitor-widget">
      <div className="widget-header">
        <span style={{ fontSize: 13, color: '#888', marginRight: 8 }}>ID: {panelId}</span>
        <a href="#" onClick={() => window.location.reload()} style={{ marginRight: '1em' }}>
          <img src={githubLogo} alt="GitHub Logo" style={{ width: '32px', height: '32px' }}/>
        </a>
      </div>

      {error && <p className="error-message">{error}</p>}

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

          <div className="view-toggle">
            <button onClick={() => setViewMode('messages')} className={viewMode === 'messages' ? 'active' : ''}>Messages</button>
            <button onClick={() => setViewMode('chart')} className={viewMode === 'chart' ? 'active' : ''}>Chart</button>
            <button onClick={() => setViewMode('nodes')} className={viewMode === 'nodes' ? 'active' : ''}>Nodes</button>
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

          {viewMode === 'nodes' && (
            <div className="nodes-view">
              {/* D3 graph container with fixed size and border for debug */}
              <div id={`d3-nodes-graph-${panelId}`} style={{ width: 800, maxWidth: '100%', height: 500, margin: '24px auto', background: '#222', border: '2px solid #646cff', borderRadius: 8, color: '#fff', display: 'flex', alignItems: 'center', justifyContent: 'center' }}>
                {/* D3.js node graph will appear here. */}
              </div>
            </div>
          )}
        </div>
      )}
    </div>
  );
}

export default RosMonitorWidget; 