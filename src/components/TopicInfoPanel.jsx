import React, { useState } from 'react';

const ExpandableItem = ({ label, data }) => {
  const [open, setOpen] = useState(false);
  return (
    <li style={{ marginBottom: 4 }}>
      <span
        style={{ cursor: 'pointer', color: '#4ecdc4', textDecoration: 'underline' }}
        onClick={() => setOpen(o => !o)}
      >
        {label}
      </span>
      {open && (
        <ul style={{ margin: '4px 0 0 12px', padding: 0, listStyle: 'none', color: '#ccc', fontSize: 12 }}>
          {Object.entries(data).map(([k, v]) => (
            <li key={k}><strong>{k}:</strong> {typeof v === 'object' ? JSON.stringify(v) : String(v)}</li>
          ))}
        </ul>
      )}
    </li>
  );
};

const TopicInfoPanel = ({ topicInfo, topicInfoLoading, fetchTopicInfo, selectedTopic }) => {
  const [showPublishers, setShowPublishers] = useState(false);
  const [showSubscribers, setShowSubscribers] = useState(false);

  return (
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
            <strong
              style={{ cursor: 'pointer', color: '#4ecdc4', textDecoration: 'underline' }}
              onClick={() => setShowPublishers(v => !v)}
            >
              Publishers {showPublishers ? '▼' : '▶'}
            </strong>
            {showPublishers && (
              <ul style={{ margin: 0, paddingLeft: 18 }}>
                {topicInfo.publishers && topicInfo.publishers.length > 0
                  ? topicInfo.publishers.map((pub, i) =>
                      typeof pub === 'object' && pub !== null
                        ? <ExpandableItem key={i} label={pub.node_name || `Publisher ${i+1}`} data={pub} />
                        : <li key={i}>{pub}</li>
                    )
                  : <li style={{ color: '#888' }}>None</li>}
              </ul>
            )}
          </div>
          <div style={{ marginBottom: '8px' }}>
            <strong
              style={{ cursor: 'pointer', color: '#4ecdc4', textDecoration: 'underline' }}
              onClick={() => setShowSubscribers(v => !v)}
            >
              Subscribers {showSubscribers ? '▼' : '▶'}
            </strong>
            {showSubscribers && (
              <ul style={{ margin: 0, paddingLeft: 18 }}>
                {topicInfo.subscribers && topicInfo.subscribers.length > 0
                  ? topicInfo.subscribers.map((sub, i) =>
                      typeof sub === 'object' && sub !== null
                        ? <ExpandableItem key={i} label={sub.node_name || `Subscriber ${i+1}`} data={sub} />
                        : <li key={i}>{sub}</li>
                    )
                  : <li style={{ color: '#888' }}>None</li>}
              </ul>
            )}
          </div>
        </div>
      ) : (
        <p style={{ color: '#888', margin: 0 }}>No topic information available</p>
      )}
    </div>
  );
};

export default TopicInfoPanel; 