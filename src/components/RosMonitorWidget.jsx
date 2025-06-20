import React, { useEffect, useState, useRef } from 'react';
import ros from '../ros/rosbridge';
import ROSLIB from 'roslib';

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

function RosMonitorWidget() {
  const [topics, setTopics] = useState([]);
  const [loading, setLoading] = useState(true);
  const [error, setError] = useState(null);
  const [selectedTopic, setSelectedTopic] = useState('');
  const [messages, setMessages] = useState([]);
  const [listener, setListener] = useState(null);
  const [filter, setFilter] = useState('');
  const [showHex, setShowHex] = useState(false);
  const timestampsRef = useRef([]); // Store timestamps of received messages

  useEffect(() => {
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
  }, []);

  useEffect(() => {
    if (listener) {
      listener.unsubscribe();
      setListener(null);
    }
    setMessages([]);
    timestampsRef.current = [];
    if (selectedTopic) {
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
          console.log('Received message:', message);
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
  }, [selectedTopic]);

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

  return (
    <div style={{ height: 'calc(100vh - 40px)', display: 'flex', padding: '20px', boxSizing: 'border-box', gap: '16px' }}>
      {/* Topics selector column */}
      <div style={{ width: 320, minWidth: 200, maxWidth: 400, background: '#222', color: '#fff', padding: 16, borderRadius: 8, display: 'flex', flexDirection: 'column' }}>
        <h3 style={{ marginTop: 0 }}>Available topics</h3>
          {loading && <div>Loading topics...</div>}
          {error && <div style={{ color: 'red' }}>{error}</div>}
          {!loading && !error && (
            <div style={{ display: 'flex', flexDirection: 'column', overflow: 'hidden', flex: 1 }}>
              <input
                type="text"
                placeholder="Filter topics..."
                value={filter}
                onChange={e => setFilter(e.target.value)}
                style={{ width: '100%', boxSizing: 'border-box', marginBottom: 10, padding: 6, borderRadius: 4, border: '1px solid #888', fontSize: 14 }}
              />
              <select
                value={selectedTopic}
                onChange={e => setSelectedTopic(e.target.value)}
                style={{ width: '100%', padding: 8, borderRadius: 4, border: '1px solid #888', fontSize: 15, background: '#fff', color: '#222', flex: 1 }}
                size={20}
              >
                <option value="" disabled>
                  {filteredTopics.length === 0 ? 'No topics found' : 'Select a topic...'}
                </option>
                {filteredTopics.map((topic) => (
                  <option key={topic} value={topic}>
                    {topic}
                  </option>
                ))}
              </select>
            </div>
          )}
      </div>
      {/* Messages column */}
      <div style={{ flex: 1, background: '#f7f7f7', color: '#222', padding: 16, borderRadius: 8, minWidth: 0, display: 'flex', flexDirection: 'column' }}>
        {selectedTopic ? (
          <>
            <div style={{ marginBottom: 8, display: 'flex', alignItems: 'center', gap: 16, flexShrink: 0 }}>
              <span style={{ fontWeight: 'bold', fontSize: 15 }}>
                Frequency: <span style={{ color: hz > 0 ? '#0a0' : '#888' }}>{hz.toFixed(2)} Hz</span>
              </span>
              <label style={{ fontSize: 14, userSelect: 'none', display: 'flex', alignItems: 'center', gap: 4 }}>
                <input type="checkbox" checked={showHex} onChange={e => setShowHex(e.target.checked)} />
                Show hex
              </label>
            </div>
            <h4 style={{ marginTop: 0, flexShrink: 0 }}>Recent messages from <code>{selectedTopic}</code>:</h4>
            {messages.length === 0 ? (
              <div style={{ color: '#888' }}>No messages received yet.</div>
            ) : (
              <ul style={{ flex: 1, overflowY: 'auto', background: '#222', color: '#fff', padding: '8px 12px', borderRadius: 4, fontFamily: 'monospace', fontSize: 13, margin: 0, listStyle: 'none' }}>
                {messages.map((msg, idx) => (
                  <li key={idx} style={{ marginBottom: 8, whiteSpace: 'pre-wrap', wordBreak: 'break-all' }}>
                    {showHex ? toHex(msg) : JSON.stringify(msg, null, 2)}
                  </li>
                ))}
              </ul>
            )}
          </>
        ) : (
          <div style={{ color: '#888', margin: 'auto' }}>Select a topic to view its messages.</div>
        )}
      </div>
    </div>
  );
}

export default RosMonitorWidget; 