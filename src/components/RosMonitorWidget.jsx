import React, { useEffect, useState, useRef } from 'react';
import rosInit from '../ros/rosbridge';
import ROSLIB from 'roslib';
import githubLogo from '../assets/github-mark.svg';
import { useRef as chartRef, useEffect as chartEffect } from 'react';
import { Chart, LineController, LineElement, PointElement, LinearScale, Title, CategoryScale } from 'chart.js';
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
  const [viewMode, setViewMode] = useState('messages'); // 'messages' or 'chart'
  const chartCanvasRef = chartRef(null);
  const chartInstanceRef = chartRef(null);
  const [numericFields, setNumericFields] = useState([]);
  const [selectedField, setSelectedField] = useState('');
  const [messageType, setMessageType] = useState('');
  const [selectedTopic, setSelectedTopic] = useState('');

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
    </div>
  );
}

export default RosMonitorWidget; 