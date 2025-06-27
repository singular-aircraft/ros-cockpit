import React from 'react';

const LogControls = ({
  showHex, setShowHex,
  showTimestamps, setShowTimestamps,
  autoScroll, setAutoScroll,
  wrapLines, setWrapLines,
  maxMessages, setMaxMessages,
  onClear,
  availableFields, selectedFields, setSelectedFields,
  fieldsDropdownOpen, setFieldsDropdownOpen
}) => (
  <div className="controls-container">
    <div style={{ display: 'flex', flexWrap: 'wrap', gap: '10px', alignItems: 'center' }}>
      <label className="control-item">
        <input type="checkbox" checked={showHex} onChange={e => setShowHex(e.target.checked)} />
        Show HEX
      </label>
      <label className="control-item">
        <input type="checkbox" checked={showTimestamps} onChange={e => setShowTimestamps(e.target.checked)} />
        Show Timestamps
      </label>
      <label className="control-item">
        <input type="checkbox" checked={autoScroll} onChange={e => setAutoScroll(e.target.checked)} />
        Auto-scroll
      </label>
      <label className="control-item">
        <input type="checkbox" checked={wrapLines} onChange={e => setWrapLines(e.target.checked)} />
        Wrap Lines
      </label>
      <div className="control-item">
        <label>Max messages:</label>
        <input
          type="number"
          value={maxMessages}
          onChange={e => setMaxMessages(Math.max(1, parseInt(e.target.value) || 100))}
          min="1"
          max="1000"
          style={{ width: '60px', padding: '2px 4px' }}
        />
      </div>
      <button
        onClick={onClear}
        className="clear-button"
      >
        Clear Log
      </button>
      {availableFields.length > 0 && (
        <div className="control-item" style={{ flexDirection: 'column', alignItems: 'flex-start', gap: '4px', position: 'relative' }}>
          <label style={{ fontSize: '11px', color: '#ccc' }}>Fields to show:</label>
          <button
            onClick={e => { e.stopPropagation(); setFieldsDropdownOpen(!fieldsDropdownOpen); }}
            style={{
              background: '#2a2a2a',
              color: '#fff',
              border: '1px solid #444',
              borderRadius: '4px',
              padding: '4px 8px',
              fontSize: '11px',
              cursor: 'pointer',
              minWidth: '150px',
              textAlign: 'left',
              display: 'flex',
              justifyContent: 'space-between',
              alignItems: 'center'
            }}
          >
            <span>{selectedFields.length} of {availableFields.length} fields</span>
            <span style={{ fontSize: '10px' }}>▼</span>
          </button>
          {fieldsDropdownOpen && (
            <div
              className="fields-dropdown"
              onClick={e => e.stopPropagation()}
              style={{
                position: 'absolute',
                top: '100%',
                left: 0,
                background: '#1a1a1a',
                border: '1px solid #444',
                borderRadius: '4px',
                padding: '8px',
                maxHeight: '200px',
                overflowY: 'auto',
                zIndex: 1000,
                minWidth: '200px',
                boxShadow: '0 4px 8px rgba(0,0,0,0.5)'
              }}
            >
              <div className="field-buttons" style={{ marginBottom: '8px', borderBottom: '1px solid #333', paddingBottom: '8px' }}>
                <button
                  onClick={() => setSelectedFields(availableFields)}
                  className="field-button field-button-all"
                >
                  Select All
                </button>
                <button
                  onClick={() => setSelectedFields([])}
                  className="field-button field-button-none"
                >
                  Select None
                </button>
              </div>
              {availableFields.map(field => (
                <label key={field} style={{
                  display: 'flex',
                  alignItems: 'center',
                  gap: '6px',
                  padding: '2px 0',
                  fontSize: '11px',
                  cursor: 'pointer',
                  color: '#fff'
                }}>
                  <input
                    type="checkbox"
                    checked={selectedFields.includes(field)}
                    onChange={e => {
                      if (e.target.checked) {
                        setSelectedFields([...selectedFields, field]);
                      } else {
                        setSelectedFields(selectedFields.filter(f => f !== field));
                      }
                    }}
                    style={{ margin: 0 }}
                  />
                  <span style={{
                    color: selectedFields.includes(field) ? '#4ecdc4' : '#ccc',
                    fontWeight: selectedFields.includes(field) ? 'bold' : 'normal'
                  }}>
                    {field}
                  </span>
                </label>
              ))}
            </div>
          )}
        </div>
      )}
    </div>
  </div>
);

export default LogControls; 