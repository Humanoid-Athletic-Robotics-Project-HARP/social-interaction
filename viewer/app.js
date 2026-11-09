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
function route() {
  const parts=location.hash.slice(1).split('/');
  if(parts.length===3&&parts[0]==='view'&&/^[a-zA-Z0-9_-]+$/.test(parts[1])&&names[parts[2]])return {source:parts[1],view:parts[2]};
  return null;
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
  return feeds(source,view).filter(([,v])=>!v.online).map(([k,v])=>`${k}: ${v.reason||'no data'}`).join('; ')||'Receiving data';
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
      const grid=node('div','service-grid');
      for(const e of items){
        const button=node('a','service-button');button.setAttribute('role','link');button.setAttribute('aria-label',`Open ${e.source.label}: ${names[e.view]}`);
        button.addEventListener('click',event=>{if(button.getAttribute('aria-disabled')==='true')event.preventDefault();});
        const top=node('span','service-top');top.append(node('span','service-machine',e.source.machine),node('span','badge','Offline'));
        button.append(top,node('span','service-title',names[e.view]),node('span','service-source',e.source.label),node('span','service-detail'),node('span','open-view','Open view'));
        grid.append(button);cards.set(e.key,button);
      }
      $('content').append(grid);
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
function render() {
  const selected=route(),view=Boolean(selected);document.body.classList.toggle('view-page',view);
  if(view)$('home').removeAttribute('aria-current');else $('home').setAttribute('aria-current','page');
  $('view-status').hidden=!view;$('footer').hidden=view;
  if(selected){showView(selected);document.title=names[selected.view]+' · HARP viewer';}
  else{$('title').textContent='HARP viewer';$('source-name').textContent='Home';document.title='HARP viewer';showHome();}
}
window.addEventListener('hashchange',render);
const events=new EventSource('/events');
events.onmessage=e=>{try{state=JSON.parse(e.data);lastState=Date.now();$('connection').textContent='Dashboard connected';render();}catch(error){console.error(error);}};
events.onerror=()=>{$('connection').textContent='Dashboard disconnected · reconnecting';};
setInterval(()=>{if(lastState&&Date.now()-lastState>4000){for(const s of state.sources){s.reachable=false;s.reason='Dashboard connection lost';}$('connection').textContent='Dashboard disconnected · reconnecting';render();}},1000);
render();
