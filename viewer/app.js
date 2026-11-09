'use strict';
const $ = id => document.getElementById(id);
const names = {lidar:'Lidar',camera:'Camera',hybrid:'Hybrid',diagnostics:'Navigation and diagnostics',speech:'Speech',recordings:'Recording',him:'HIM simulator'};
let state = {sources:[]}, lastState = 0, signature = '';
const cards = new Map();
function node(tag,cls,text) { const n=document.createElement(tag); if(cls)n.className=cls; if(text!==undefined)n.textContent=text; return n; }
function views(source) {
  if(source.kind==='him')return ['him'];
  if(source.kind==='speech')return ['speech'];
  if(source.kind==='simulation')return ['diagnostics'];
  if(source.kind==='recording')return ['recordings'];
  return source.layout==='hybrid'?['lidar','camera','hybrid']:[source.layout];
}
const ID=/^[a-zA-Z0-9_-]+$/;
function route() {
  const parts=location.hash.slice(1).split('/');
  if(parts.length===3&&parts[0]==='view'&&ID.test(parts[1])&&names[parts[2]])return {source:parts[1],view:parts[2]};
  if(parts.length===1&&parts[0]==='runs')return {page:'runs'};
  if(parts.length===2&&parts[0]==='timing'&&RUN_NAME.test(decodeURIComponent(parts[1])))return {page:'timing',name:decodeURIComponent(parts[1])};
  if(parts.length===3&&parts[0]==='compare'&&ID.test(parts[1])&&ID.test(parts[2]))return {page:'compare',ids:[parts[1],parts[2]]};
  return null;
}
async function post(path,body) {
  const response=await fetch(path,{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(body)});
  const value=await response.json();if(!response.ok)throw new Error(value.error||response.status);return value;
}
function feeds(source,view) {
  const keys={lidar:['lidar'],camera:['depth','colour'],hybrid:['lidar','depth'],speech:['speech'],diagnostics:['diagnostics'],recordings:['diagnostics'],him:['viewer']}[view];
  return keys.map(k=>[k,source.feeds[k]||{online:false,reason:'Waiting for data'}]);
}
function health(source,view) {
  if(!source.reachable)return ['', 'Offline'];
  if(source.kind==='recording')return ['replay','Replay'];
  if(source.kind==='him')return ['live','Ready'];
  const f=feeds(source,view), live=f.filter(([,v])=>v.online).length;
  if(!live)return ['', 'Offline'];
  if(live<f.length)return ['partial','Partial'];
  return ['live',source.kind==='speech'&&!source.detail.running?'Idle':'Live'];
}
function reason(source,view) {
  if(!source.reachable)return source.reason||'Waiting for this service.';
  const missing=feeds(source,view).filter(([,v])=>!v.online).map(([k,v])=>`${k}: ${v.reason||'no data'}`).join('; ');
  // A run says which avoidance filter, detector and loop it has (their start events).
  const pieces=(source.detail.pieces||[]).join(' · ');
  // Deadline health of a sim run: HIM stops it once a state comes 0.2 s late.
  const d=source.detail,late=d.late_states?`${d.late_states} late state${d.late_states>1?'s':''} (max ${d.max_late_s} s)`:'';
  const stopped=d.stopped?`HIM stopped it: ${d.stopped}`:'';
  return [stopped,missing,pieces,late].filter(Boolean).join(' · ')||'Receiving data';
}
function showHost() {
  // Whether this computer is likely to keep a sim run's deadlines (submission/host_health.py).
  const h=state.host,line=$('host');
  if(!h||route()){line.hidden=true;return;}
  // Each reading as a chip, with its target when it falls short.
  const readings=[],t=h.targets||{};
  if(h.free_percent!=null)readings.push([`${h.free_percent}% memory free`,h.free_percent<t.free_percent?`aim for ${t.free_percent}%+`:'']);
  else if(h.free_mb!=null)readings.push([`${(h.free_mb/1000).toFixed(1)} GB available`,'']);
  if(h.compressed_mb!=null)readings.push([`${(h.compressed_mb/1000).toFixed(1)} GB compressed`,t.compressed_mb!=null&&h.compressed_mb>t.compressed_mb?`aim for ${(t.compressed_mb/1000).toFixed(1)} GB or less`:'']);
  if(h.viewers)readings.push([`${h.viewers} HIM viewer${h.viewers>1?'s':''} open`,'close after use']);
  if(h.pressure&&h.pressure!=='normal')readings.push([`macOS: ${h.pressure} pressure`,'']);
  const verdict=h.ready===true?'Ready for a sim run':h.ready===false?'A sim run may stop on a deadline':'Sim deadlines not measured here';
  const key=verdict+JSON.stringify(readings);
  if(line.dataset.key!==key){
    line.dataset.key=key;
    line.replaceChildren(node('span','host-dot'),node('span','host-name','This computer'),node('span','host-verdict',verdict),
      ...readings.map(([text,aim])=>{const chip=node('span','host-reading',text);if(aim)chip.append(node('em','',aim));return chip;}));
  }
  line.title=h.ready===false?'Quit apps you don\'t need, close HIM viewers, and restart Docker Desktop between batches (context/sim-deadline-stops.md).':'';
  line.className='host'+(h.ready===false?' warn':h.ready==null?' unknown':'');line.hidden=false;
}
function entries() { return state.sources.flatMap(s=>views(s).map(view=>({source:s,view,key:s.id+'/'+view}))); }
function priority(entry) {
  const [status]=health(entry.source,entry.view);
  return status==='live'||status==='replay'?0:status==='partial'?1:entry.source.reachable?2:3;
}
function showHome() {
  const items=entries(),key='home:'+Boolean(lastState)+':'+items.map(e=>e.key+':'+e.source.label).join('|');
  if(key!==signature){
    signature=key;cards.clear();$('content').replaceChildren();
    if(!items.length){
      const empty=node('div','empty');empty.append(node('h3','',lastState?'No services configured':'Connecting to dashboard'),node('p','',lastState?'Add services to the dashboard source configuration.':'Waiting for service status.'));$('content').append(empty);
    }else{
      // Running now: a square per reachable process, with a few live facts.
      const live=node('section','live');live.setAttribute('aria-label','Running now');
      live.append(node('h2','section-title','Running now'),node('div','live-grid'));
      const all=node('section','services');all.setAttribute('aria-label','All services');
      all.append(node('h2','section-title','All services'));
      $('content').append(live,all);
      const grid=node('div','service-grid');
      // Saved runs first: a list of course.py's run folders, any of them a replay.
      const runs=node('a','service-button runs-card');runs.href='#runs';runs.setAttribute('aria-label','Open saved runs');
      const runsTop=node('span','service-top');runsTop.append(node('span','service-machine','This computer'),node('span','badge replay','Runs'));
      runs.append(runsTop,node('span','service-title','Saved runs'),node('span','service-source','Replay or compare any run'),node('span','service-detail runs-detail'),node('span','open-view','Open list'));
      grid.append(runs);
      for(const e of items){
        const button=node('a','service-button');button.setAttribute('role','link');button.setAttribute('aria-label',`Open ${e.source.label}: ${names[e.view]}`);
        button.addEventListener('click',event=>{if(button.getAttribute('aria-disabled')==='true')event.preventDefault();});
        const top=node('span','service-top');top.append(node('span','service-machine',e.source.machine),node('span','badge','Offline'));
        button.append(top,node('span','service-title',names[e.view]),node('span','service-source',e.source.label),node('span','service-detail'),node('span','open-view','Open view'));
        grid.append(button);cards.set(e.key,button);
      }
      all.append(grid);
    }
  }
  for(const e of items){
    const button=cards.get(e.key);if(!button)continue;
    const [cls,label]=health(e.source,e.view),badge=button.querySelector('.badge');badge.className='badge '+cls;badge.textContent=label;
    const available=e.source.reachable;
    button.setAttribute('aria-disabled',String(!available));
    if(available){button.href='#view/'+e.key;button.removeAttribute('tabindex');}
    else{button.removeAttribute('href');button.tabIndex=-1;}
    const detail=button.querySelector('.service-detail'),message=reason(e.source,e.view);
    detail.textContent=message;detail.title=message;
  }
  drawLive();
  const runsDetail=$('content').querySelector('.runs-detail');
  if(runsDetail)runsDetail.textContent=state.runs==null?'':`${state.runs} runs in runs/`+((state.opened||[]).length?` · ${state.opened.length} open`:'');
  const grid=$('content').querySelector('.service-grid');
  if(grid){
    const sorted=items.slice().sort((a,b)=>priority(a)-priority(b)),order=sorted.map(e=>e.key).join('|');
    if(grid.dataset.order!==order){
      const focused=document.activeElement;
      for(const e of sorted){const card=cards.get(e.key);if(grid.moveBefore)grid.moveBefore(card,null);else grid.append(card);}
      if(focused&&grid.contains(focused)&&document.activeElement!==focused)focused.focus({preventScroll:true});
      grid.dataset.order=order;
    }
  }
}
const kinds={simulation:'Simulation',recording:'Replay',speech:'Speech',sensors:'Robot sensors',him:'HIM simulator'};
function plural(n,word){return `${n} ${word}${n===1?'':'s'}`;}
function facts(source) {
  // Two to four short lines about what the process is doing now.
  const d=source.detail||{},out=[];
  if(source.kind==='simulation'||source.kind==='recording'){
    if(d.stopped)out.push(`HIM stopped it: ${d.stopped}`);
    if((d.pieces||[]).length)out.push(d.pieces.join(' · '));
    out.push(d.late_states?`${plural(d.late_states,'late state')} (max ${Math.round(d.max_late_s*1000)} ms)`:'No late states');
    const last=(d.events||[]).filter(e=>!['odom','timing','late_state'].includes(e.event)).at(-1);
    if(last)out.push(`Last: ${String(last.event).replace(/_/g,' ')}`);
    if(source.kind==='simulation'&&d.frame_age!=null&&d.frame_age>=4)out.push(`No view frames for ${Math.round(d.frame_age)} s`);
    if(d.renderer&&d.renderer!=='ready')out.push(`Renderer: ${d.renderer}`);
  }else if(source.kind==='speech'){
    out.push(d.running?'Conversation running':'Idle, not listening');
    out.push(`LLM ${d.llm==='ok'?'ready':d.llm||'unknown'}`);
    if(d.config)out.push([d.config.listen?`hears ${d.config.source}`:'',d.config.talk?`speaks with ${d.config.voice}`:''].filter(Boolean).join(' · '));
    if(d.other_listen)out.push('Another listen.py is running');
  }else if(source.kind==='sensors'){
    const l=d.lidar,c=d.camera;
    if(l)out.push(l.connected?`Lidar on${l.pps?` · ${Math.round(l.pps).toLocaleString()} pts/s`:''}`:`Lidar off: ${l.why||'no data'}`);
    if(c)out.push(c.depth_connected?`Depth ${c.depth_fps??'–'} fps${c.valid!=null?` · ${c.valid}% filled`:''}`:`Depth off: ${c.depth_why||'no data'}`);
    if(c)out.push(c.colour_connected?'Colour on':'Colour off');
  }else if(source.kind==='him'){
    out.push('Viewer ready','Close its launcher after use: it keeps every frame');
  }
  return out.filter(Boolean).slice(0,4);
}
function primary(source){const v=views(source);return v.includes('hybrid')?'hybrid':v[0];}
function drawLive() {
  const box=$('content').querySelector('.live-grid');if(!box)return;
  const running=state.sources.filter(s=>s.reachable),key=running.map(s=>s.id).join('|')+':'+Boolean(lastState);
  if(box.dataset.key!==key){
    box.dataset.key=key;box.replaceChildren();
    if(!running.length){
      const empty=node('div','live-empty');
      empty.append(node('strong','',lastState?'Nothing running':'Connecting'),node('span','',lastState?'Start a sim run or speech/web.py and it shows here.':'Waiting for service status.'));
      box.append(empty);
    }
    for(const s of running){
      const tile=node('a','live-tile'),view=primary(s);tile.href=`#view/${s.id}/${view}`;tile.dataset.id=s.id;
      tile.setAttribute('aria-label',`Open ${s.label}: ${names[view]}`);
      const top=node('span','live-top');top.append(node('span','badge'),node('span','live-machine',s.machine));
      tile.append(top,node('span','live-kind',kinds[s.kind]||names[view]),node('span','live-label',s.label===kinds[s.kind]?'':s.label),node('ul','live-facts'),node('span','live-open',views(s).length>1?`Open ${names[view].toLowerCase()} · ${views(s).length} views`:'Open'));
      box.append(tile);
    }
  }
  for(const tile of box.querySelectorAll('.live-tile')){
    const s=state.sources.find(x=>x.id===tile.dataset.id);if(!s)continue;
    const [cls,label]=health(s,primary(s)),badge=tile.querySelector('.badge');badge.className='badge '+cls;badge.textContent=label;
    tile.classList.toggle('stopped',Boolean(s.detail&&s.detail.stopped));
    const lines=facts(s),list=tile.querySelector('.live-facts'),text=lines.join('\n');
    if(list.dataset.text!==text){list.dataset.text=text;list.replaceChildren(...lines.map(t=>{const li=node('li','',t);li.title=t;return li;}));}
  }
}
function showView(selected) {
  const source=state.sources.find(s=>s.id===selected.source),valid=source&&views(source).includes(selected.view);
  $('title').textContent=names[selected.view];$('source-name').textContent=source?`${source.label} · ${source.machine}`:selected.source;
  const key='view:'+selected.source+':'+selected.view+':'+Boolean(lastState)+':'+Boolean(valid);
  if(key!==signature){
    signature=key;cards.clear();$('content').replaceChildren();
    const body=node('div','view-body'),overlay=node('div','offline');overlay.setAttribute('role','status');
    overlay.append(node('strong','','Offline'),node('p','','Waiting for data.'));body.append(overlay);$('content').append(body);
  }
  const body=$('content').querySelector('.view-body'),overlay=body.querySelector('.offline');
  if(!valid){
    $('view-status').textContent=lastState?'Unavailable':'Connecting';$('view-status').className='badge';
    overlay.hidden=false;overlay.querySelector('strong').textContent=lastState?'Service unavailable':'Connecting to dashboard';overlay.querySelector('p').textContent=lastState?'This view is not in the current source configuration. Return Home to choose a service.':'Waiting for service status.';return;
  }
  const [cls,label]=health(source,selected.view);$('view-status').className='badge '+cls;$('view-status').textContent=label;
  const ready=source.reachable&&(selected.view==='diagnostics'||selected.view==='recordings'||feeds(source,selected.view).some(([,f])=>f.online));
  overlay.hidden=ready;overlay.querySelector('strong').textContent='Offline';overlay.querySelector('p').textContent=reason(source,selected.view);body.classList.toggle('stale',!ready);
  // Load a reachable service behind the cover: the camera may start colour lazily.
  if(source.reachable&&!body.querySelector('iframe')){
    const frame=node('iframe');frame.title=`${names[selected.view]} · ${source.label}`;
    frame.src=['diagnostics','recordings','him'].includes(selected.view)?source.detail.viewer:`/source/${encodeURIComponent(source.id)}/${selected.view}/`;body.prepend(frame);
  }
}
let runList=null,runsAt=0,picked=[];
const when=name=>name.slice(0,10)+' '+name.slice(11,16).replace('-',':');
function outcome(run) {
  if(!run.scored)return ['','Not scored'];
  if(run.outcome==='succeeded'&&run.passed!==false)return ['live','Passed'];
  if(run.outcome==='inconclusive')return ['','Inconclusive'];
  const word=run.passed===false&&run.outcome==='succeeded'?'Failed checks':run.outcome||'failed';
  return ['fail',word[0].toUpperCase()+word.slice(1)];
}
async function loadRuns(force) {
  if(!force&&runList&&Date.now()-runsAt<10000)return;
  runsAt=Date.now();
  let fresh;
  try{fresh=(await (await fetch('/api/runs')).json()).runs;}catch(error){fresh=runList||[];}
  // Redraw only on a change: a redraw would drop focus and the pointer's place.
  if(runList&&JSON.stringify(fresh)===JSON.stringify(runList))return;
  runList=fresh;
  if(route()?.page==='runs')drawRuns();
}
async function openRuns(names_) {
  try{
    const ids=[];for(const name of names_)ids.push((await post('/api/runs/open',{name})).id);
    location.hash=ids.length===1?`#view/${ids[0]}/recordings`:`#compare/${ids[0]}/${ids[1]}`;
  }catch(error){alert('Could not open the run: '+error.message);}
}
function showRuns() {
  $('title').textContent='Saved runs';$('source-name').textContent='Replay one, or pick two to compare';document.title='Saved runs · HARP viewer';
  if(signature!=='runs'){
    signature='runs';cards.clear();
    const page=node('div','runs-page'),bar=node('div','runs-bar');
    const filter=node('input','runs-filter');filter.type='search';filter.placeholder='Filter by world, tag, outcome or piece';filter.id='runs-filter';
    filter.addEventListener('input',drawRuns);
    const compare=node('button','runs-compare','Compare');compare.id='runs-compare';compare.disabled=true;
    compare.addEventListener('click',()=>openRuns(picked));
    bar.append(filter,compare);page.append(bar,node('p','runs-open'),node('div','runs-list'));
    $('content').replaceChildren(page);drawRuns();loadRuns(true);
  }
  const opened=(state.opened||[]).map(id=>state.sources.find(s=>s.id===id)).filter(Boolean);
  const line=$('content').querySelector('.runs-open'),key=opened.map(s=>s.id).join('|');
  if(line.dataset.key!==key){
    line.dataset.key=key;line.replaceChildren();
    if(opened.length){
      line.append(node('span','','Open now (each holds a Rerun worker): '));
      for(const s of opened){
        const chip=node('span','open-chip'),link=node('a','',s.label);link.href=`#view/${s.id}/recordings`;
        const close=node('button','','Close');close.addEventListener('click',()=>post('/api/runs/close',{id:s.id}));
        chip.append(link,close);line.append(chip);
      }
    }
  }
  loadRuns(false);
}
function drawRuns() {
  const list=$('content').querySelector('.runs-list');if(!list)return;
  if(!runList){list.replaceChildren(node('p','','Reading runs/ …'));return;}
  const words=($('runs-filter').value||'').toLowerCase().split(/\s+/).filter(Boolean);
  const shown=runList.filter(r=>{const text=[r.name,r.world,r.tag,outcome(r)[1],r.him_reason,...r.pieces].join(' ').toLowerCase();return words.every(w=>text.includes(w));});
  picked=picked.filter(name=>runList.some(r=>r.name===name));
  const rows=shown.slice(0,300).map(r=>{
    const row=node('div','run-row'),[cls,label]=outcome(r);
    const pick=node('input');pick.type='checkbox';pick.checked=picked.includes(r.name);pick.disabled=!r.replay;pick.setAttribute('aria-label','Compare '+r.name);
    pick.addEventListener('change',()=>{picked=pick.checked?[...picked.filter(n=>n!==r.name),r.name].slice(-2):picked.filter(n=>n!==r.name);drawRuns();});
    const main=node('div','run-main');main.append(node('span','run-world',r.world||'no world'),node('span','run-tag',r.tag||''),node('span','run-when',when(r.name)));
    const facts=[r.him_reason?`HIM stopped it: ${r.him_reason}`:'',r.seconds!=null?`${r.seconds} s`:'',r.closest_m!=null?`closest ${r.closest_m} m`:'',
                 r.contacts?`${r.contacts} contacts`:'',...r.pieces,r.commit?`${r.commit}${r.dirty?' (changed)':''}`:''].filter(Boolean);
    const detail=node('div','run-detail',facts.join(' · '));detail.title=r.failed_checks&&r.failed_checks.length?'Failed: '+r.failed_checks.join(', '):'';
    const replay=node('button','run-replay',r.replay?'Replay':'No replay');replay.disabled=!r.replay;replay.addEventListener('click',()=>openRuns([r.name]));
    // State timing against HIM's deadline, for runs with harp_node.py's timing lines.
    const timing=node('a','run-timing','Timing');if(r.timing)timing.href='#timing/'+encodeURIComponent(r.name);else{timing.setAttribute('aria-disabled','true');timing.title='No timing lines in this run';}
    const actions=node('div','run-actions');actions.append(timing,replay);
    row.append(pick,main,node('span','badge '+cls,label),detail,actions);return row;
  });
  list.replaceChildren(...(rows.length?rows:[node('p','','No runs match.')]));
  if(shown.length>300)list.append(node('p','',`Showing the newest 300 of ${shown.length}: filter to narrow.`));
  const compare=$('runs-compare');compare.disabled=picked.length!==2;compare.textContent=`Compare (${picked.length}/2)`;
}
function showCompare(selected) {
  const sources=selected.ids.map(id=>state.sources.find(s=>s.id===id));
  $('title').textContent='Compare runs';$('source-name').textContent=sources.map((s,i)=>s?s.label:selected.ids[i]).join('  vs  ');document.title='Compare runs · HARP viewer';
  const key='compare:'+selected.ids.join('|')+':'+sources.map(s=>Boolean(s&&s.detail.viewer)).join();
  $('view-status').textContent='Replay';$('view-status').className='badge replay';
  if(key===signature)return;
  signature=key;cards.clear();
  const pair=node('div','compare');
  sources.forEach((s,i)=>{
    const pane=node('div','compare-pane');
    // The embedded replay names its run itself; say so only when it is missing.
    if(s&&s.detail.viewer){const frame=node('iframe');frame.title=s.label;frame.src=s.detail.viewer;pane.append(frame);}
    else pane.append(node('div','compare-label','Not open: return to Saved runs and pick it again'));
    pair.append(pane);
  });
  $('content').replaceChildren(pair);
}
function render() {
  const selected=route(),full=Boolean(selected&&selected.page!=='runs'&&selected.page!=='timing');document.body.classList.toggle('view-page',full);
  if(selected)$('home').removeAttribute('aria-current');else $('home').setAttribute('aria-current','page');
  $('view-status').hidden=!full;$('footer').hidden=Boolean(selected);
  if(selected?.page==='runs')showRuns();
  else if(selected?.page==='timing')showTiming(selected);
  else if(selected?.page==='compare')showCompare(selected);
  else if(selected){showView(selected);document.title=names[selected.view]+' · HARP viewer';}
  else{$('title').textContent='HARP viewer';$('source-name').textContent='Home';document.title='HARP viewer';showHome();}
  showHost();
}
window.addEventListener('hashchange',render);
const events=new EventSource('/events');
events.onmessage=e=>{try{state=JSON.parse(e.data);lastState=Date.now();$('connection').textContent='Dashboard connected';render();}catch(error){console.error(error);}};
events.onerror=()=>{$('connection').textContent='Dashboard disconnected · reconnecting';};
setInterval(()=>{if(lastState&&Date.now()-lastState>4000){for(const s of state.sources){s.reachable=false;s.reason='Dashboard connection lost';}$('connection').textContent='Dashboard disconnected · reconnecting';render();}},1000);
render();
