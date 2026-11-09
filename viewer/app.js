'use strict';
const $ = id => document.getElementById(id);
const tabs = {
  overview: ['Overview', 'Every source connects independently.'],
  lidar: ['Lidar', 'Point clouds and coverage, kept separate by source.'],
  camera: ['Camera', 'Colour, depth, measurements and the top-down map.'],
  hybrid: ['Hybrid', 'Lidar and camera from the same sensor source.'],
  diagnostics: ['Navigation and diagnostics', 'Our simulation feeds and Rerun diagnostics.'],
  speech: ['Speech', 'Listening and conversations on each source machine.'],
  recordings: ['Recordings', 'Saved diagnostics, with their own recording timelines.']
};
let state = {sources: []}, active = 'overview', selection = 'all', signature = '';
let lastState = 0;
const cards = new Map();
function node(tag, cls, text) { const n = document.createElement(tag); if (cls) n.className = cls; if (text !== undefined) n.textContent = text; return n; }
function sourceFeeds(s) { return Object.entries(s.feeds); }
function health(s) {
  const f = sourceFeeds(s), live = f.filter(([,v]) => v.online).length;
  if (s.kind === 'recording' && s.reachable) return ['replay', 'Replay'];
  if (!s.reachable) return ['', 'Offline'];
  if (live === 0) return ['', 'Offline'];
  if (live < f.length) return ['partial', 'Partial'];
  return ['live', s.kind === 'speech' && !s.detail.running ? 'Idle' : 'Live'];
}
function matches(s) {
  if (active === 'overview') return true;
  if (active === 'speech') return s.kind === 'speech';
  if (active === 'recordings') return s.kind === 'recording';
  if (active === 'diagnostics') return s.kind === 'simulation';
  return s.kind === 'sensors' && (s.layout === 'hybrid' || s.layout === active);
}
function canShow(s) {
  if (!s.reachable) return false;
  if (active === 'speech') return !!s.feeds.speech?.online;
  if (active === 'lidar') return !!s.feeds.lidar?.online;
  if (active === 'camera') return !!(s.feeds.depth?.online || s.feeds.colour?.online);
  if (active === 'hybrid') return !!(s.feeds.lidar?.online || s.feeds.depth?.online);
  return !!s.detail.viewer;
}
function reason(s) {
  if (!s.reachable) return s.reason || 'Waiting for this source.';
  return sourceFeeds(s).filter(([,v]) => !v.online).map(([k,v]) => `${k}: ${v.reason || 'no data'}`).join('; ') || 'Waiting for a viewer feed.';
}
function header(s) {
  const h = node('div','card-header'), names = node('div');
  names.append(node('h3','',s.label),node('div','machine',`${s.machine} · ${s.id}`));
  h.append(names,node('span','badge','Offline')); return h;
}
function render() {
  $('title').textContent = tabs[active][0]; $('description').textContent = tabs[active][1];
  document.querySelectorAll('#tabs button').forEach(b => b.setAttribute('aria-selected',String(b.dataset.tab === active)));
  const all = state.sources.filter(matches), visible = all.filter(s => selection === 'all' || selection === s.id);
  const filterKey = all.map(s => `${s.id}:${s.label}`).join('|');
  if ($('filter').dataset.key !== filterKey) {
    $('filter').replaceChildren(new Option('All sources','all'), ...all.map(s => new Option(s.label,s.id)));
    if (!all.some(s => s.id === selection)) selection = 'all';
    $('filter').value = selection; $('filter').dataset.key = filterKey;
  }
  $('filter-label').hidden = active === 'overview' || all.length < 2;
  $('summary').replaceChildren(...state.sources.map(s => {
    const [cls,label] = health(s), p = node('span','pill'); p.append(node('span','dot '+cls),document.createTextNode(`${s.label}: ${label}`)); return p;
  }));
  const key = active + ':' + selection + ':' + Boolean(lastState) + ':' + visible.map(s => s.id).join('|');
  if (key !== signature) {
    signature = key; cards.clear(); $('content').replaceChildren();
    if (!visible.length) {
      const empty = node('div','empty');
      const title = !lastState ? 'Connecting to dashboard' : active === 'recordings' ? 'No recordings added' : 'No source configured for this view';
      const detail = !lastState ? 'Waiting for source status.' : active === 'recordings' ? 'Add a saved run or .rrd capture to the source configuration to replay it here.' : 'Add a source for this view to the dashboard configuration. Other sources connect independently.';
      empty.append(node('h3','',title),node('p','',detail)); $('content').append(empty);
    } else {
      const grid = node('div', active === 'overview' ? 'grid' : 'views' + (visible.length > 1 ? ' multi' : ''));
      for (const s of visible) {
        const card = node('section','card'); card.append(header(s));
        if (active === 'overview') card.append(node('div','feed-list'),node('div','age'));
        else {
          const body = node('div','view-body'), overlay = node('div','offline'); overlay.append(node('strong','','Offline'),node('p','','Waiting for data.')); body.append(overlay); card.append(body);
          card.append(node('div','source-note',active === 'speech' ? `Audio runs on ${s.machine}. ${s.controls ? 'Speech controls enabled.' : 'Observation only.'}` : 'Source: '+s.id));
        }
        grid.append(card); cards.set(s.id,card);
      }
      $('content').append(grid);
    }
  }
  for (const s of visible) {
    const card = cards.get(s.id); if (!card) continue;
    const [cls,label] = health(s), badge = card.querySelector('.badge'); badge.className = 'badge '+cls; badge.textContent = label;
    if (active === 'overview') {
      const rows = sourceFeeds(s).map(([name,f]) => {
        const row = node('div','feed-row'), text = node('div','',name); text.append(node('small','',f.reason || (f.online ? 'Receiving data' : 'No data'))); row.append(text,node('span','',f.online ? 'Online' : 'Offline')); return row;
      });
      if (!rows.length) { const r = node('div','feed-row',reason(s)); rows.push(r); }
      card.querySelector('.feed-list').replaceChildren(...rows);
      card.querySelector('.age').textContent = s.age === null ? 'No update received' : `Last status ${s.age.toFixed(1)} s ago`;
    } else {
      const body = card.querySelector('.view-body'), overlay = body.querySelector('.offline');
      const ready = canShow(s); overlay.hidden = ready; overlay.querySelector('p').textContent = reason(s); body.classList.toggle('stale',!ready);
      // A reachable sensor service may start colour lazily on the first image
      // request. Load its page behind the offline cover even before data arrives.
      if (s.reachable && !body.querySelector('iframe')) {
        const frame = node('iframe'); frame.title = `${tabs[active][0]} · ${s.label}`;
        frame.src = active === 'diagnostics' || active === 'recordings' ? s.detail.viewer : `/source/${encodeURIComponent(s.id)}/${active}/`;
        body.prepend(frame);
      }
    }
  }
}
for (const [id,[label]] of Object.entries(tabs)) {
  const b = node('button','',label); b.dataset.tab = id; b.setAttribute('role','tab'); b.onclick = () => { active = id; selection = 'all'; location.hash = id; render(); }; $('tabs').append(b);
}
$('tabs').setAttribute('role','tablist');
$('filter').onchange = () => { selection = $('filter').value; render(); };
window.addEventListener('hashchange', () => { const name = location.hash.slice(1); if (tabs[name]) { active = name; render(); } });
if (tabs[location.hash.slice(1)]) active = location.hash.slice(1);
const events = new EventSource('/events');
events.onmessage = e => { try { state = JSON.parse(e.data); lastState = Date.now(); $('connection').textContent = 'Dashboard connected'; render(); } catch (error) { console.error(error); } };
events.onerror = () => { $('connection').textContent = 'Dashboard disconnected · reconnecting'; };
setInterval(() => {
  if (lastState && Date.now()-lastState > 4000) {
    for (const s of state.sources) { s.reachable = false; s.reason = 'Dashboard connection lost'; }
    $('connection').textContent = 'Dashboard disconnected · reconnecting'; render();
  }
},1000);
render();
