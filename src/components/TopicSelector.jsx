import React from 'react';

const TopicSelector = ({
  topics,
  filter,
  setFilter,
  selectedTopic,
  handleTopicChange,
  loading,
  filteredTopics
}) => (
  <div className="topic-selection">
    <input
      type="text"
      placeholder="Filter topics..."
      value={filter}
      onChange={e => setFilter(e.target.value)}
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
);

export default TopicSelector; 