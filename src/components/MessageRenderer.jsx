import React from 'react';

function toHex(val) {
  if (typeof val === 'number') {
    return '0x' + val.toString(16);
  }
  if (typeof val === 'string') {
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
    let out = '{ ';
    for (const k in val) {
      out += `${k}: ${toHex(val[k])}, `;
    }
    return out + '}';
  }
  return String(val);
}

const MessageRenderer = ({ data, showHex }) => {
  if (data === null || typeof data === 'undefined') {
    return <span style={{ color: '#888' }}>null</span>;
  }
  if (typeof data !== 'object' || data === null) {
    return <span className="message-field-value">{showHex ? toHex(data) : String(data)}</span>;
  }
  if (Array.isArray(data)) {
    if (data.length > 10) {
      return <span style={{ color: '#888' }}>[Array length: {data.length}]</span>;
    }
    return (
      <span>
        <span className="message-bracket">[</span>
        {data.map((item, index) => (
          <span key={index}>
            <MessageRenderer data={item} showHex={showHex} />
            {index < data.length - 1 && <span className="message-comma">, </span>}
          </span>
        ))}
        <span className="message-bracket">]</span>
      </span>
    );
  }
  return (
    <span className="message-content">
      <span className="message-brace">{'{'}</span>
      {Object.entries(data).map(([key, value], index) => (
        <span key={key}>
          <span className="message-field-key">{key}</span>
          <span className="message-field-separator">: </span>
          <MessageRenderer data={value} showHex={showHex} />
          {index < Object.keys(data).length - 1 && <span className="message-comma">, </span>}
        </span>
      ))}
      <span className="message-brace">{'}'}</span>
    </span>
  );
};

export default MessageRenderer; 