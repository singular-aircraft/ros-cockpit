import React from 'react';
import MessageRenderer from './MessageRenderer';

const LogView = ({
  messages,
  showTimestamps,
  showHex,
  wrapLines,
  messagesEndRef,
  selectedFields,
  filterMessageByFields
}) => (
  <div className="messages-log-container" style={{
    background: '#1a1a1a',
    border: '1px solid #444',
    borderRadius: '4px',
    padding: '8px',
    maxHeight: '400px',
    overflowY: 'auto',
    fontFamily: 'monospace',
    fontSize: '12px'
  }}>
    {messages.length === 0 ? (
      <p style={{ color: '#888', textAlign: 'center', margin: '20px 0' }}>
        Waiting for messages...
      </p>
    ) : (
      <div>
        {messages.map((msg, index) => (
          <div
            key={`${msg.id}-${index}`}
            className="message-entry"
            style={{
              wordBreak: wrapLines ? 'break-word' : 'normal'
            }}
          >
            <div style={{ display: 'flex', alignItems: 'flex-start', gap: '8px' }}>
              {showTimestamps && (
                <span className="timestamp" style={{
                  color: '#4ecdc4',
                  fontSize: '11px',
                  fontFamily: 'monospace',
                  whiteSpace: 'nowrap',
                  flexShrink: 0
                }}>
                  [{msg.timestamp.toLocaleTimeString()}]
                </span>
              )}
              <div style={{ flex: 1, minWidth: 0 }}>
                <div
                  className={wrapLines ? '' : 'message-no-wrap'}
                  style={{
                    whiteSpace: wrapLines ? 'pre-wrap' : 'nowrap',
                    overflow: wrapLines ? 'visible' : 'hidden',
                    textOverflow: wrapLines ? 'clip' : 'ellipsis'
                  }}
                  title={wrapLines ? '' : 'Hover to see full message'}
                >
                  <MessageRenderer
                    data={selectedFields.length > 0 ? filterMessageByFields(msg.data, selectedFields) : msg.data}
                    showHex={showHex}
                  />
                </div>
              </div>
            </div>
          </div>
        ))}
        <div ref={messagesEndRef} />
      </div>
    )}
  </div>
);

export default LogView; 