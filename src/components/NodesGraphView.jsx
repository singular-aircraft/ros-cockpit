import React, { useEffect, useRef } from 'react';
import * as d3 from 'd3';

const NodesGraphView = ({
  panelId,
  graphServiceData,
  fetchGraphInfo,
  graphServiceLoading,
  graphServiceError
}) => {
  const d3SimRef = useRef(null);
  const d3NodesRef = useRef([]);
  const d3LinksRef = useRef([]);
  const d3SvgRef = useRef(null);
  const d3GRef = useRef(null);

  useEffect(() => {
    console.log('Efecto D3 ejecutado', { graphServiceData, panelId });
    const containerId = `d3-nodes-graph-${panelId}`;
    const container = document.getElementById(containerId);
    console.log('containerId:', containerId, 'container:', container);
    if (!graphServiceData) return;
    if (!container) return;

    // Build nodes and links from graphServiceData
    let allNodes = [];
    let links = [];
    const TOPIC_FILTER = ['/rosout', '/parameter_events'];
    const isFiltered = name => TOPIC_FILTER.includes(name);
    allNodes = [
      ...(graphServiceData.nodes || []).map(n => ({ id: n.name, type: 'node' })),
      ...(graphServiceData.topics || []).filter(t => !isFiltered(t.name)).map(t => ({ id: t.name, type: 'topic' })),
      ...(graphServiceData.services || []).filter(s => !isFiltered(s.name)).map(s => ({ id: s.name, type: 'service' })),
    ];
    (graphServiceData.publishers || []).forEach(pub => {
      if (!isFiltered(pub.topic))
        links.push({ source: pub.node, target: pub.topic, type: 'publish' });
    });
    (graphServiceData.subscribers || []).forEach(sub => {
      if (!isFiltered(sub.topic))
        links.push({ source: sub.topic, target: sub.node, type: 'subscribe' });
    });
    if (graphServiceData.service_providers) {
      graphServiceData.service_providers.forEach(sp => {
        if (!isFiltered(sp.service))
          links.push({ source: sp.node, target: sp.service, type: 'service' });
      });
    }
    const nodeList = allNodes;
    const width = container.offsetWidth || 600;
    const height = container.offsetHeight || 400;

    // Limpio todos los SVGs antes de crear uno nuevo
    Array.from(container.querySelectorAll('svg')).forEach(svgEl => svgEl.remove());
    let svg = d3SvgRef.current;
    let g = d3GRef.current;
    if (!svg) {
      svg = d3.select(container)
        .append('svg')
        .attr('width', width)
        .attr('height', height)
        .style('background', '#222');
      d3SvgRef.current = svg;
      g = svg.append('g');
      d3GRef.current = g;
      svg.call(d3.zoom()
        .scaleExtent([0.1, 2])
        .on('zoom', (event) => {
          g.attr('transform', event.transform);
        })
      );
    }

    // --- Mantener posiciones previas ---
    const prevNodes = d3NodesRef.current;
    const prevNodeMap = Object.fromEntries(prevNodes.map(n => [n.id, n]));
    nodeList.forEach(n => {
      const prev = prevNodeMap[n.id];
      if (prev) {
        n.x = prev.x;
        n.y = prev.y;
        n.vx = prev.vx;
        n.vy = prev.vy;
        n.fx = prev.fx;
        n.fy = prev.fy;
      }
    });
    d3NodesRef.current = nodeList;
    d3LinksRef.current = links;

    // --- Definir marcador de flecha ---
    if (svg.select('defs').empty()) {
      const defs = svg.append('defs');
      defs.append('marker')
        .attr('id', 'arrowhead')
        .attr('viewBox', '0 -5 10 10')
        .attr('refX', 18)
        .attr('refY', 0)
        .attr('markerWidth', 8)
        .attr('markerHeight', 8)
        .attr('orient', 'auto')
        .attr('markerUnits', 'strokeWidth')
        .append('path')
        .attr('d', 'M0,-5L10,0L0,5')
        .attr('fill', '#ffd93d');
    }
    // --- Data join para links ---
    links = links.map(l => ({
      ...l,
      source: typeof l.source === 'object' ? l.source.id : l.source,
      target: typeof l.target === 'object' ? l.target.id : l.target
    }));
    const linkSel = g.selectAll('line').data(links, d => d.source + '-' + d.target + '-' + d.type);
    linkSel.exit().remove();
    linkSel.enter()
      .append('line')
      .attr('stroke', '#ffd93d')
      .attr('stroke-width', 10)
      .attr('stroke-dasharray', d => d.type === 'service' ? '4,2' : '')
      .attr('marker-end', 'url(#arrowhead)');
    // --- Data join para nodos ---
    const nodeSel = g.selectAll('circle').data(nodeList, d => d.id);
    nodeSel.exit().remove();
    nodeSel.enter()
      .append('circle')
      .attr('stroke', '#fff')
      .attr('stroke-width', 2)
      .attr('r', d => d.type === 'node' ? 44 : d.type === 'topic' ? 32 : 24)
      .attr('fill', d => d.type === 'node' ? '#2ecc40' : d.type === 'topic' ? '#43e' : '#fa0')
      .call(d3.drag()
        .on('start', dragstarted)
        .on('drag', dragged)
        .on('end', dragended));
    // --- Data join para labels ---
    const labelSel = g.selectAll('text').data(nodeList, d => d.id);
    labelSel.exit().remove();
    labelSel.enter()
      .append('text')
      .attr('fill', '#fff')
      .attr('font-size', 14)
      .attr('text-anchor', 'middle')
      .attr('dy', 5)
      .text(d => d.id);
    // --- Simulación de fuerzas ---
    if (!d3SimRef.current) {
      d3SimRef.current = d3.forceSimulation(nodeList)
        .force('link', d3.forceLink(links).id(d => d.id).distance(120))
        .force('charge', d3.forceManyBody().strength(-400))
        .force('center', d3.forceCenter(width / 2, height / 2))
        .on('tick', ticked);
    } else {
      d3SimRef.current.nodes(nodeList);
      d3SimRef.current.force('link').links(links);
      d3SimRef.current.alpha(1).restart();
    }
    function ticked() {
      // Repulsión extra entre topics
      const topicNodes = d3NodesRef.current.filter(n => n.type === 'topic');
      const repelStrength = 0.35;
      for (let i = 0; i < topicNodes.length; i++) {
        for (let j = i + 1; j < topicNodes.length; j++) {
          const a = topicNodes[i];
          const b = topicNodes[j];
          let dx = a.x - b.x;
          let dy = a.y - b.y;
          let dist = Math.sqrt(dx * dx + dy * dy) || 1;
          if (dist < 200) {
            let force = repelStrength / dist;
            a.vx += force * dx;
            a.vy += force * dy;
            b.vx -= force * dx;
            b.vy -= force * dy;
          }
        }
      }
      g.selectAll('line')
        .attr('x1', d => (typeof d.source === 'object' ? d.source.x : d3NodesRef.current.find(n => n.id === d.source)?.x))
        .attr('y1', d => (typeof d.source === 'object' ? d.source.y : d3NodesRef.current.find(n => n.id === d.source)?.y))
        .attr('x2', d => (typeof d.target === 'object' ? d.target.x : d3NodesRef.current.find(n => n.id === d.target)?.x))
        .attr('y2', d => (typeof d.target === 'object' ? d.target.y : d3NodesRef.current.find(n => n.id === d.target)?.y));
      g.selectAll('circle')
        .attr('cx', d => d.x)
        .attr('cy', d => d.y);
      g.selectAll('text')
        .attr('x', d => d.x)
        .attr('y', d => d.y);
    }
    function dragstarted(event, d) {
      if (!event.active && d3SimRef.current) d3SimRef.current.alphaTarget(0.3).restart();
      d.fx = d.x;
      d.fy = d.y;
    }
    function dragged(event, d) {
      d.fx = event.x;
      d.fy = event.y;
    }
    function dragended(event, d) {
      if (!event.active && d3SimRef.current) d3SimRef.current.alphaTarget(0);
      d.fx = null;
      d.fy = null;
    }
    // Cleanup on unmount
    return () => {
      if (d3SimRef.current) d3SimRef.current.stop();
      d3SimRef.current = null;
      // Elimino todos los SVGs del contenedor
      if (container) Array.from(container.querySelectorAll('svg')).forEach(svgEl => svgEl.remove());
      d3SvgRef.current = null;
      d3GRef.current = null;
    };
  }, [graphServiceData, panelId]);

  return (
    <div className="nodes-view">
      <div style={{ marginBottom: 12 }}>
        <button onClick={fetchGraphInfo} disabled={graphServiceLoading} style={{ background: '#4ecdc4', color: '#000', border: 'none', borderRadius: 4, padding: '4px 12px', fontSize: 13, cursor: graphServiceLoading ? 'not-allowed' : 'pointer', opacity: graphServiceLoading ? 0.6 : 1 }}>
          {graphServiceLoading ? 'Loading graph info...' : 'Fetch Graph Info (Service)'}
        </button>
      </div>
      {graphServiceError && <div style={{ color: '#ff6b6b', marginBottom: 8 }}>Error: {graphServiceError}</div>}
      <div id={`d3-nodes-graph-${panelId}`} style={{ width: 800, maxWidth: '100%', height: 500, margin: '24px auto', background: '#222', border: '2px solid #646cff', borderRadius: 8, color: '#fff', display: 'flex', alignItems: 'center', justifyContent: 'center' }}>
        {/* D3 node graph will appear here */}
      </div>
    </div>
  );
};

export default NodesGraphView;