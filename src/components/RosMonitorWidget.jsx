import React, { useEffect, useState, useRef, useCallback } from 'react';
import rosInit from '../ros/rosbridge';
import ROSLIB from 'roslib';
import githubLogo from '../assets/github-mark.svg';
import { useRef as chartRef, useEffect as chartEffect } from 'react';
import { Chart, LineController, LineElement, PointElement, LinearScale, Title, CategoryScale } from 'chart.js';
import * as d3 from 'd3';
import MessageRenderer from './MessageRenderer';
import TopicSelector from './TopicSelector';
import LogControls from './LogControls';
import LogView from './LogView';
import TopicInfoPanel from './TopicInfoPanel';
import ChartView from './ChartView';
import NodesGraphView from './NodesGraphView';
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

function RosMonitorWidget({ panelId, host, viewMode: initialViewMode }) {
  const [topics, setTopics] = useState([]);
  const [loading, setLoading] = useState(true);
  const [error, setError] = useState(null);
  const [messages, setMessages] = useState([]);
  const [latestMessage, setLatestMessage] = useState(null);
  const [listener, setListener] = useState(null);
  const [filter, setFilter] = useState('');
  const timestampsRef = useRef([]);
  const [showHex, setShowHex] = useState(false);
  const [isConnected, setIsConnected] = useState(false);
  const rosRef = useRef(null);
  const maxMessagesRef = useRef(1000); // Store maxMessages in ref to avoid re-renders
  const messagesRef = useRef([]); // Store messages in ref to avoid re-renders
  const [viewMode, setViewMode] = useState(initialViewMode === 'nodes' ? 'nodes' : 'messages');

  useEffect(() => {
    if (!host) return;

    try {
      const ros = rosInit(host);
      rosRef.current = ros;
      
      ros.on('connection', () => {
        console.log('Connected to ROS');
        setIsConnected(true);
        setError(null);
      });

      ros.on('close', () => {
        console.log('Disconnected from ROS');
        setIsConnected(false);
        setError('Connection closed');
      });

      ros.on('error', (error) => {
        console.error('ROS error:', error);
        setIsConnected(false);
        setError(`Connection error: ${error.message || 'Unknown error'}`);
      });

      // Cleanup
      return () => {
        if (rosRef.current) {
          rosRef.current.close();
          rosRef.current = null;
        }
      };
    } catch (error) {
      console.error('Failed to initialize ROS connection:', error);
      setIsConnected(false);
      setError(`Failed to connect: ${error.message || 'Invalid host'}`);
    }
  }, [host]);
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
  // Nuevos estados para el log
  const [maxMessages, setMaxMessages] = useState(100);
  const [showTimestamps, setShowTimestamps] = useState(true);
  const [autoScroll, setAutoScroll] = useState(true);
  const [wrapLines, setWrapLines] = useState(false);
  const [selectedFields, setSelectedFields] = useState([]);
  const [availableFields, setAvailableFields] = useState([]);
  const [fieldsDropdownOpen, setFieldsDropdownOpen] = useState(false);
  const messagesEndRef = useRef(null);
  const logContainerRef = useRef(null);
  const latestMessageRef = useRef(null);

  // Función para hacer scroll automático al final del log
  const scrollToBottom = () => {
    if (autoScroll && logContainerRef.current) {
      logContainerRef.current.scrollTop = logContainerRef.current.scrollHeight;
    }
  };

  // Efecto para scroll automático
  useEffect(() => {
    if (messages.length > 0) {
      scrollToBottom();
    }
  }, [messages, autoScroll]);

  // Al montar, lee el topic guardado
  useEffect(() => {
    if (!host || !isConnected) return;
    
    const ros = rosRef.current;
    if (!ros) return;

    ros.getTopics((result) => {
      const list = Array.isArray(result) ? result : (Array.isArray(result?.topics) ? result.topics : []);
      setTopics(list);
      setLoading(false);
    });
  }, [host, isConnected]);

  useEffect(() => {
    if (!panelId) {
      return;
    }
    const key = `rosmonitor_panel_topic_${panelId}`;
    const saved = localStorage.getItem(key);
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
    if (!topicName || !host) return;

    setTopicInfoLoading(true);

    const ros = rosRef.current || rosInit(host);
    const topicInfoService = new ROSLIB.Service({
      ros: ros,
      name: '/get_topic_info',
      serviceType: 'ros2_monitor_srvs/GetTopicInfo'
    });

    const request = new ROSLIB.ServiceRequest({
      topic_name: topicName
    });

    topicInfoService.callService(request, (result) => {
      if (result && result.success) {
        const infoStr = result.raw_info || result.message || result.info || '';
        let parsedInfo;
        try {
          parsedInfo = typeof infoStr === 'string' ? JSON.parse(infoStr) : infoStr;
        } catch (err) {
          parsedInfo = null;
        }
        if (parsedInfo && typeof parsedInfo === 'object') {
          setTopicInfo({
            type: parsedInfo.type || parsedInfo.msg_type || 'std_msgs/String',
            publishers: parsedInfo.publishers || [],
            subscribers: parsedInfo.subscribers || [],
            publisherCount: parsedInfo.publisher_count || parsedInfo.publisherCount || 0,
            subscriberCount: parsedInfo.subscriber_count || parsedInfo.subscriberCount || 0,
            rawInfo: parsedInfo
          });
        } else {
          // could not parse JSON, keep raw text
          setTopicInfo({
            type: 'Unknown',
            publishers: [],
            subscribers: [],
            publisherCount: 0,
            subscriberCount: 0,
            rawInfo: infoStr,
            error: 'Could not parse topic information, showing raw'
          });
        }
      } else {
        // Fallback to basic info
        fetchBasicTopicInfo(topicName, ros);
      }
      setTopicInfoLoading(false);
    }, (error) => {
      // Fallback to basic info
      fetchBasicTopicInfo(topicName, ros);
      setTopicInfoLoading(false);
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
        type: type,
        publishers: ['Publisher info not available via WebSocket'],
        subscribers: ['Subscriber info not available via WebSocket'],
        publisherCount: 0,
        subscriberCount: 0,
        note: 'For detailed publisher/subscriber info, use "ros2 topic info --verbose" in terminal'
      };
      
      setTopicInfo(topicInfo);
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
        const handleMessage = (message) => {
          const timestamp = new Date();
          const messageWithTimestamp = {
            data: message,
            timestamp: timestamp,
            id: Date.now() + Math.random()
          };
          latestMessageRef.current = message;
          messagesRef.current = [...messagesRef.current, messageWithTimestamp].slice(-maxMessagesRef.current);
          const now = Date.now();
          timestampsRef.current = [now, ...timestampsRef.current].slice(0, 10);
          setLatestMessage(message);
          setMessages([...messagesRef.current]);
        };
        newListener.subscribe(handleMessage);
        setListener(newListener);
      });
    }
    return () => {
      if (listener) listener.unsubscribe();
    };
  }, [selectedTopic, host, maxMessages]);

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
  }, [selectedTopic, host, viewMode, panelId]);

  // Chart rendering effect
  chartEffect(() => {
    if (viewMode !== 'chart' || !chartCanvasRef.current || !selectedField) return;
    // Extract values for the chart
    const getValue = (msg, path) => {
      // Soporta path tipo 'data[0]' para arrays
      const parts = path.split('.');
      let val = msg.data || msg; // Acceder a msg.data si existe, sino usar msg directamente
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

  // Efecto para cerrar el desplegable cuando se hace clic fuera
  useEffect(() => {
    const handleClickOutside = (event) => {
      if (fieldsDropdownOpen) {
        setFieldsDropdownOpen(false);
      }
    };

    if (fieldsDropdownOpen) {
      document.addEventListener('click', handleClickOutside);
    }

    return () => {
      document.removeEventListener('click', handleClickOutside);
    };
  }, [fieldsDropdownOpen]);

  // Función para extraer todos los campos disponibles de un mensaje
  const extractAvailableFields = (data, prefix = '') => {
    const fields = [];
    
    if (data === null || typeof data === 'undefined') {
      return fields;
    }
    
    if (typeof data !== 'object') {
      return fields;
    }
    
    if (Array.isArray(data)) {
      if (data.length > 0) {
        // Para arrays, extraer campos del primer elemento
        const arrayFields = extractAvailableFields(data[0], prefix ? `${prefix}[0]` : '[0]');
        fields.push(...arrayFields);
      }
      return fields;
    }
    
    // Para objetos, extraer todos los campos
    Object.entries(data).forEach(([key, value]) => {
      const fieldPath = prefix ? `${prefix}.${key}` : key;
      fields.push(fieldPath);
      
      // Recursivamente extraer campos de sub-objetos
      if (typeof value === 'object' && value !== null && !Array.isArray(value)) {
        const subFields = extractAvailableFields(value, fieldPath);
        fields.push(...subFields);
      }
    });
    
    return fields;
  };

  // Actualizar campos disponibles cuando cambie el mensaje más reciente
  useEffect(() => {
    if (latestMessage) {
      const fields = extractAvailableFields(latestMessage);
      setAvailableFields(fields);
      
      // Si no hay campos seleccionados, seleccionar todos por defecto
      if (selectedFields.length === 0) {
        setSelectedFields(fields);
      }
    }
  }, [latestMessage]);

  // Función para filtrar mensaje según campos seleccionados
  const filterMessageByFields = (data, fields) => {
    if (fields.length === 0) return data;
    
    const filtered = {};
    fields.forEach(field => {
      const value = getValueFromPath(data, field);
      if (value !== undefined) {
        setValueAtPath(filtered, field, value);
      }
    });
    
    return filtered;
  };

  // Función auxiliar para obtener valor de una ruta
  const getValueFromPath = (obj, path) => {
    const parts = path.split('.');
    let current = obj;
    
    for (const part of parts) {
      if (current === null || current === undefined) return undefined;
      
      const arrayMatch = part.match(/^(\w+)\[(\d+)\]$/);
      if (arrayMatch) {
        const [, key, index] = arrayMatch;
        current = current[key];
        if (Array.isArray(current)) {
          current = current[parseInt(index)];
        } else {
          return undefined;
        }
      } else {
        current = current[part];
      }
    }
    
    return current;
  };

  // Función auxiliar para establecer valor en una ruta
  const setValueAtPath = (obj, path, value) => {
    const parts = path.split('.');
    let current = obj;
    
    for (let i = 0; i < parts.length - 1; i++) {
      const part = parts[i];
      const arrayMatch = part.match(/^(\w+)\[(\d+)\]$/);
      
      if (arrayMatch) {
        const [, key, index] = arrayMatch;
        if (!current[key]) current[key] = [];
        if (!current[key][parseInt(index)]) current[key][parseInt(index)] = {};
        current = current[key][parseInt(index)];
      } else {
        if (!current[part]) current[part] = {};
        current = current[part];
      }
    }
    
    const lastPart = parts[parts.length - 1];
    const arrayMatch = lastPart.match(/^(\w+)\[(\d+)\]$/);
    
    if (arrayMatch) {
      const [, key, index] = arrayMatch;
      if (!current[key]) current[key] = [];
      current[key][parseInt(index)] = value;
    } else {
      current[lastPart] = value;
    }
  };

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
    console.log('Llamando a /get_graph_info...');
    graphService.callService(request, (result) => {
      try {
        console.log('Respuesta de /get_graph_info:', result);
        const data = JSON.parse(result.graph_json);
        setGraphServiceData(data);
        console.log('setGraphServiceData ejecutado con:', data);
      } catch (e) {
        setGraphServiceError('Error parsing graph_json: ' + e);
      }
      setGraphServiceLoading(false);
    }, (err) => {
      setGraphServiceError('Service call failed: ' + (err?.toString() || 'unknown error'));
      setGraphServiceLoading(false);
    });
  };

  useEffect(() => {
    if (viewMode !== 'nodes' || !isConnected) return;
    fetchGraphInfo();
  }, [viewMode, host, isConnected]);

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
          <TopicSelector
            topics={topics}
            filter={filter}
            setFilter={setFilter}
            selectedTopic={selectedTopic}
            handleTopicChange={handleTopicChange}
            loading={loading}
            filteredTopics={filteredTopics}
          />
          {selectedTopic && (
            <div className="topic-info">
              <h3>
                Topic: {selectedTopic}
              </h3>
              <TopicInfoPanel
                topicInfo={topicInfo}
                topicInfoLoading={topicInfoLoading}
                fetchTopicInfo={fetchTopicInfo}
                selectedTopic={selectedTopic}
              />
              <LogControls
                showHex={showHex}
                setShowHex={setShowHex}
                showTimestamps={showTimestamps}
                setShowTimestamps={setShowTimestamps}
                autoScroll={autoScroll}
                setAutoScroll={setAutoScroll}
                wrapLines={wrapLines}
                setWrapLines={setWrapLines}
                maxMessages={maxMessages}
                setMaxMessages={setMaxMessages}
                onClear={() => setMessages([])}
                availableFields={availableFields}
                selectedFields={selectedFields}
                setSelectedFields={setSelectedFields}
                fieldsDropdownOpen={fieldsDropdownOpen}
                setFieldsDropdownOpen={setFieldsDropdownOpen}
              />
              <LogView
                messages={messages}
                showTimestamps={showTimestamps}
                showHex={showHex}
                wrapLines={wrapLines}
                messagesEndRef={messagesEndRef}
                selectedFields={selectedFields}
                filterMessageByFields={filterMessageByFields}
              />
              {viewMode === 'chart' && (
                <ChartView
                  messages={messages}
                  numericFields={numericFields}
                  selectedField={selectedField}
                  setSelectedField={setSelectedField}
                  chartCanvasRef={chartCanvasRef}
                  selectedTopic={selectedTopic}
                  messageType={messageType}
                  chartInstanceRef={chartInstanceRef}
                />
              )}
            </div>
          )}
        </>
      )}

      {/* Only show nodes view if in nodes mode */}
      {isNodesMode && (
        <NodesGraphView
          panelId={panelId}
          graphServiceData={graphServiceData}
          graphServiceLoading={graphServiceLoading}
          graphServiceError={graphServiceError}
        />
      )}
    </div>
  );
}

export default RosMonitorWidget; 