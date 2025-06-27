import React, { useEffect } from 'react';

const ChartView = ({
  messages,
  numericFields,
  selectedField,
  setSelectedField,
  chartCanvasRef,
  selectedTopic,
  messageType,
  chartInstanceRef
}) => {
  useEffect(() => {
    if (!selectedField || !chartCanvasRef.current) return;
    // Extract values for the chart
    const getValue = (msg, path) => {
      const parts = path.split('.');
      let val = msg.data || msg;
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
    // eslint-disable-next-line no-undef
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
        animation: { duration: 200 },
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
          legend: { labels: { color: '#ddd' } },
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
  }, [messages, selectedField, chartCanvasRef, selectedTopic]);

  return (
    <div className="chart-view">
      {numericFields.length > 0 ? (
        <>
          <select onChange={e => setSelectedField(e.target.value)} value={selectedField}>
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
  );
};

export default ChartView; 