import React, { useRef, useState, useEffect } from 'react';
import { DockviewReact } from 'dockview-react';
import 'dockview-react/dist/styles/dockview.css';
import RosMonitorWidget from './components/RosMonitorWidget';

const LAYOUT_KEY = 'dockview_layout_v1';

function App() {
  const dockviewApiRef = useRef(null);
  const panelId = useRef(1);
  const [host, setHost] = useState(() => localStorage.getItem('rosbridge_host') || 'ws://127.0.0.1:9090');
  const [isConnected, setIsConnected] = useState(false);
  const rosRef = useRef(null);

  useEffect(() => {
    localStorage.setItem('rosbridge_host', host);
  }, [host]);

  // Manage the global connection to rosbridge
  useEffect(() => {
    let ros;
    let isUnmounted = false;
    import('./ros/rosbridge').then((mod) => {
      ros = mod.default(host);
      rosRef.current = ros;
      setIsConnected(false);
      ros.on('connection', () => { if (!isUnmounted) setIsConnected(true); });
      ros.on('close', () => { if (!isUnmounted) setIsConnected(false); });
      ros.on('error', () => { if (!isUnmounted) setIsConnected(false); });
    });
    return () => {
      isUnmounted = true;
      if (rosRef.current && rosRef.current.close) rosRef.current.close();
    };
  }, [host]);

  // Restore saved layout on startup
  const handleReady = (api) => {
    const dockApi = api.api || api;
    dockviewApiRef.current = dockApi;
    // Restore layout if it exists
    const saved = localStorage.getItem(LAYOUT_KEY);
    if (saved) {
      try {
        dockApi.fromJSON(JSON.parse(saved));
        // Ensure all panels have panelId in params
        (dockApi.panels || []).forEach((p) => {
          if (!p.params) p.params = {};
          if (!p.params.panelId) {
            p.params.panelId = p.id;
          }
        });
        // Update panelId to avoid collisions
        const maxId = (dockApi.panels || []).reduce((max, p) => {
          const n = parseInt((p.id || '').replace('monitor-', ''));
          return isNaN(n) ? max : Math.max(max, n);
        }, 0);
        panelId.current = maxId + 1;
      } catch (e) {
        // If there is an error, clear corrupted layout
        localStorage.removeItem(LAYOUT_KEY);
      }
    } else {
      // If there is no layout, create an initial panel
      dockApi.addPanel({
        id: `monitor-${panelId.current}`,
        title: `Monitor ${panelId.current}`,
        component: 'rosmonitor',
        params: {
          panelId: `monitor-${panelId.current}`,
        },
      });
      panelId.current += 1;
    }
    // Save layout on every change
    dockApi.onDidLayoutChange(() => {
      const layout = dockApi.toJSON();
      localStorage.setItem(LAYOUT_KEY, JSON.stringify(layout));
    });
  };

  const addMonitor = () => {
    const dockApi = dockviewApiRef.current;
    if (dockApi && dockApi.addPanel) {
      dockApi.addPanel({
        id: `monitor-${panelId.current}`,
        title: `Monitor ${panelId.current}`,
        component: 'rosmonitor',
        params: {
          panelId: `monitor-${panelId.current}`,
        },
      });
      panelId.current += 1;
    }
  };

  return (
    <div style={{ height: '100vh', width: '100vw', display: 'flex', flexDirection: 'column' }}>
      {/* Top header */}
      <div style={{ height: 56, background: '#23272f', display: 'flex', alignItems: 'center', justifyContent: 'space-between', padding: '0 24px', boxShadow: '0 2px 8px #0002', zIndex: 30 }}>
        <div style={{ fontWeight: 700, fontSize: 22, color: '#fff', letterSpacing: 1 }}>ROS2 Monitor</div>
        <div style={{ display: 'flex', alignItems: 'center', gap: 16 }}>
          <input
            type="text"
            value={host}
            onChange={e => setHost(e.target.value)}
            placeholder="ws://127.0.0.1:9090"
            style={{
              width: 260,
              borderRadius: 4,
              border: '1px solid #888',
              fontSize: 15,
              padding: '6px 10px',
              background: '#181a20',
              color: '#fff',
              textAlign: 'left',
            }}
            title="rosbridge host"
          />
          <span style={{ display: 'inline-block', width: 14, height: 14, borderRadius: '50%', background: isConnected ? '#0a0' : '#c00', border: '1px solid #222', marginRight: 6 }}></span>
          <span style={{ color: isConnected ? '#0a0' : '#c00', fontWeight: 600, fontSize: 15 }}>
            {isConnected ? 'Connected' : 'Disconnected'}
          </span>
          <button
            onClick={addMonitor}
            style={{
              width: 32,
              height: 32,
              borderRadius: 8,
              background: '#646cff',
              color: '#fff',
              border: 'none',
              fontSize: 22,
              fontWeight: 'bold',
              marginLeft: 8,
              cursor: 'pointer',
              boxShadow: '0 2px 8px #0003',
              transition: 'background 0.2s',
              display: 'flex',
              alignItems: 'center',
              justifyContent: 'center',
            }}
            title="Add monitor"
          >
            +
          </button>
        </div>
      </div>
      {/* Main content: dockview */}
      <div style={{ flex: 1, display: 'flex', minHeight: 0 }}>
        <div style={{ flex: 1, height: '100%' }}>
          <DockviewReact
            onReady={handleReady}
            components={{
              rosmonitor: (props) => {
                const panelId = props.id || props.params?.id || props.params?.panelId;
                return (
                  <RosMonitorWidget
                    key={props.id}
                    panelId={panelId}
                    host={host}
                  />
                );
              },
            }}
            style={{ height: '100%', width: '100%' }}
            disableFloatingGroups={false}
            showHeaderActions={true}
          />
        </div>
      </div>
    </div>
  );
}

export default App;
