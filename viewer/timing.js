'use strict';
// A saved run's state timing (#timing/RUN): each second's worst state age, gap between states,
// our reply and republishing (harp_node.py's timing lines) against HIM's 0.2 s deadline, each
// late state, and this computer's memory those seconds (run.json host_memory), on one time axis.
// Loaded before app.js, whose helpers ($, node, signature, openRuns) it uses only when called.
const RUN_NAME=/^\d{4}-\d{2}-\d{2}_\d{2}-\d{2}-\d{2}Z_[A-Za-z0-9_.-]{1,120}$/;
const SERIES=[
  {key:'age',name:'State age',label:'age',hint:'HIM receiving the state to it reaching us'},
  {key:'gap',name:'Gap between states',label:'gap',hint:'time since the state before'},
  {key:'reply',name:'Our reply',label:'reply',hint:'our joint command'},
  {key:'extra',name:'Republishing',label:'republish',hint:'republishing the state'},
];
let timingData=null,timingWidth=0,timingObserver=null;
const SVG='http://www.w3.org/2000/svg';
function svg(tag,attrs,text){const n=document.createElementNS(SVG,tag);for(const k in attrs)n.setAttribute(k,attrs[k]);if(text!==undefined)n.textContent=text;return n;}
const fmt=(v,unit)=>v==null?'–':`${Math.round(v*10)/10} ${unit}`;
function niceMax(v,step){return Math.max(step,Math.ceil(v/step)*step);}

function showTiming(selected) {
  $('title').textContent='State timing';document.title='State timing · HARP viewer';
  const key='timing:'+selected.name;
  if(signature===key)return;
  signature=key;cards.clear();timingData=null;
  $('source-name').textContent=selected.name;
  const page=node('div','timing-page');
  const head=node('div','timing-head'),back=node('a','home-link','Saved runs');back.href='#runs';
  const replay=node('button','','Replay');replay.disabled=true;replay.addEventListener('click',()=>openRuns([selected.name]));
  head.append(node('p','timing-verdict','Reading the run …'),back,replay);
  page.append(head,node('div','timing-stats'),node('div','timing-legend'),node('div','timing-chart'),node('div','timing-table'));
  $('content').replaceChildren(page);
  fetch('/api/runs/timing?name='+encodeURIComponent(selected.name)).then(r=>r.ok?r.json():Promise.reject(new Error(r.status===404?'No such run':'HTTP '+r.status)))
    .then(data=>{if(signature!==key)return;timingData=data;replay.disabled=!data.replay;drawTiming();})
    .catch(error=>{if(signature===key)head.querySelector('.timing-verdict').textContent='Could not read the run: '+error.message;});
  if(timingObserver)timingObserver.disconnect();
  timingObserver=new ResizeObserver(()=>{const w=page.querySelector('.timing-chart').clientWidth;if(timingData&&w!==timingWidth)drawChart();});
  timingObserver.observe(page.querySelector('.timing-chart'));
}

function worst(rows,key){let best=null;for(const r of rows)if(r[key]!=null&&(best==null||r[key]>best))best=r[key];return best;}

function drawTiming() {
  const d=timingData,page=$('content').querySelector('.timing-page');if(!d||!page)return;
  $('source-name').textContent=[d.world,d.tag,d.name.slice(0,16).replace('_',' ')].filter(Boolean).join(' · ');
  const age=worst(d.seconds,'age'),reply=worst(d.seconds,'reply'),lateAge=worst(d.late,'age');
  const missed=d.late.reduce((n,l)=>n+(l.missed||0),0),oldest=Math.max(age??0,lateAge??0);
  // Which part of the deadline went: ours (the reply) or the state reaching us.
  let verdict;
  if(!d.seconds.length)verdict='This run has no timing lines: harp_node.py writes them only in sim runs with our policy node.';
  else if(!d.late.length)verdict=`No state came ${d.late_ms} ms late: the oldest was ${fmt(oldest,'ms')} of HIM's ${d.deadline_ms} ms deadline.`;
  else verdict=`${d.late.length} state${d.late.length>1?'s':''} came ${d.late_ms} ms or more late (oldest ${fmt(oldest,'ms')} of ${d.deadline_ms} ms). `+
    (reply!=null&&reply<20?`Our reply stayed at ${fmt(reply,'ms')} or less, so the late time was the state reaching us: compare the memory charts below.`:`Our reply reached ${fmt(reply,'ms')}: look at our node too.`);
  page.querySelector('.timing-verdict').textContent=verdict;
  const stop=d.him_reason?'Stopped by HIM':d.outcome?d.outcome[0].toUpperCase()+d.outcome.slice(1):'Not scored';
  const ran=[d.him_reason,d.end!=null?`${Math.round(d.end)} s of policy`:''].filter(Boolean).join(' · ');
  const tiles=[['Oldest state',fmt(oldest,'ms'),`deadline ${d.deadline_ms} ms`],['Late states',String(d.late.length),missed?`${missed} missed`:`${d.late_ms} ms or more`],
    ['Our worst reply',fmt(reply,'ms'),'joint command'],['Most paged in',fmt(d.host.most_decompressed_mb_in_1s,'MB'),'in one second, whole run'],
    ['Least free memory',fmt(d.host.min_free_mb,'MB'),d.host.max_compressed_mb!=null?`${(d.host.max_compressed_mb/1000).toFixed(1)} GB compressed at most`:''],['Run',stop,ran]];
  page.querySelector('.timing-stats').replaceChildren(...tiles.map(([label,value,sub])=>{const t=node('div','timing-tile');t.append(node('span','tile-label',label),node('span','tile-value',value),node('span','tile-sub',sub));return t;}));
  page.querySelector('.timing-legend').replaceChildren(...SERIES.map(s=>{const k=node('span','legend-item');k.title=s.hint;k.append(node('span','legend-key '+s.key),node('span','',s.name));return k;}));
  drawChart();drawTable();
}

function drawChart() {
  const d=timingData,box=$('content').querySelector('.timing-chart');if(!d||!box)return;
  timingWidth=box.clientWidth;box.replaceChildren();
  if(!d.seconds.length&&!d.late.length){box.append(node('p','','Nothing to chart.'));return;}
  const W=Math.max(300,timingWidth),narrow=W<560,L=narrow?40:48,R=narrow?14:100,end=Math.max(d.end||1,...d.seconds.map(s=>s.t));
  const panels=[{title:'Each second\'s worst (ms)',h:narrow?200:250},{title:'This computer paged in each second (MB)',h:84},{title:'This computer\'s free memory (MB)',h:84}];
  let y0=0;for(const p of panels){p.top=y0+22;y0=p.top+p.h+16;}
  const H=y0+18,x=t=>L+(Math.min(Math.max(t,0),end)/end)*(W-L-R);
  const root=svg('svg',{viewBox:`0 0 ${W} ${H}`,width:W,height:H,role:'img','aria-label':'State timing and host memory over the run; the table below lists each late state.'});
  // Panel scales: timing always shows the deadline; memory starts at zero.
  const tMax=niceMax(Math.max(d.deadline_ms*1.1,...d.seconds.flatMap(s=>SERIES.map(k=>s[k.key]||0)),...d.late.map(l=>l.age||0))*1.05,50);
  // Memory scales count only the seconds drawn (the run's samples start before our policy).
  const shown=d.memory.filter(m=>m.t>=0&&m.t<=end);
  const step=(max,most)=>{for(const s of [10,20,25,50,100,200,250,500,1000,2000,2500,5000,10000])if(max/s<=most)return s;return Math.ceil(max/most);};
  const raws=[tMax,Math.max(d.paged_in_mb*1.6,...shown.map(m=>m.paged_mb||0)),Math.max(1000,...shown.map(m=>m.free_mb||0))];
  panels.forEach((p,i)=>{p.step=step(raws[i],i?2:5);p.max=i?Math.ceil(raws[i]/p.step)*p.step:raws[i];p.y=v=>p.top+p.h-(v/p.max)*p.h;});
  const [,pMax,fMax]=panels.map(p=>p.max);
  panels.forEach((p,i)=>{
    root.append(svg('text',{x:L,y:p.top-8,class:'chart-title'},p.title));
    for(let v=0;v<=p.max+1e-6;v+=p.step){
      root.append(svg('line',{x1:L,x2:W-R,y1:p.y(v),y2:p.y(v),class:v?'grid':'baseline'}));
      root.append(svg('text',{x:L-6,y:p.y(v)+4,class:'tick','text-anchor':'end'},i===2&&v>=1000?`${v/1000}k`:String(v)));
    }
  });
  // Thresholds the reader measures against, labelled where they are.
  const labels=[],ref=(p,v,label)=>{root.append(svg('line',{x1:L,x2:W-R,y1:p.y(v),y2:p.y(v),class:'reference'}));labels.push(svg('text',{x:L+6,y:p.y(v)-5,class:'ref-label'},label));};
  ref(panels[0],d.deadline_ms,`HIM's deadline ${d.deadline_ms} ms`);ref(panels[0],d.late_ms,`late ${d.late_ms} ms`);ref(panels[1],d.paged_in_mb,`${d.paged_in_mb} MB`);
  // Time ticks shared by all panels.
  const tStep=[5,10,15,20,30,60,120,300].find(s=>end/s<=(narrow?5:10))||600;
  for(let t=0;t<=end+1e-6;t+=tStep)root.append(svg('text',{x:x(t),y:H-4,class:'tick','text-anchor':'middle'},`${t} s`));
  // Timing lines, broken where a value is missing.
  const tp=panels[0],ends=[];
  for(const s of SERIES){
    let path='',pen=false,last=null;
    for(const r of d.seconds){if(r[s.key]==null){pen=false;continue;}path+=(pen?'L':'M')+x(r.t).toFixed(1)+' '+tp.y(r[s.key]).toFixed(1);pen=true;last=r;}
    root.append(svg('path',{d:path,class:'series '+s.key}));
    if(last)ends.push({s,x:x(last.t),y:tp.y(last[s.key])});
  }
  // Direct labels at the line ends, pushed apart with a leader back to each line.
  ends.sort((a,b)=>a.y-b.y);
  for(let i=1;i<ends.length;i++)ends[i].ly=Math.max(ends[i].y,(ends[i-1].ly??ends[i-1].y)+13);
  if(ends.length)ends[0].ly=ends[0].ly??ends[0].y;
  const over=(ends.at(-1)?.ly??0)-(tp.top+tp.h);if(over>0)for(const e of ends)e.ly-=over;
  for(const e of ends){
    const lx=W-R+6+(narrow?0:28);
    if(!narrow)root.append(svg('path',{d:`M${e.x+3} ${e.y}L${W-R+4} ${e.y}L${lx-3} ${e.ly}`,class:'leader '+e.s.key}));
    if(!narrow)root.append(svg('text',{x:lx,y:e.ly+4,class:'end-label'},e.s.label));
  }
  // Each late state as a ringed dot on the age scale.
  for(const l of d.late)if(l.age!=null)root.append(svg('circle',{cx:x(l.t),cy:tp.y(Math.min(l.age,tMax)),r:4.5,class:'late-dot'}));
  // Paged-in columns, 2px apart, rounded at the top only.
  const pp=panels[1],bw=Math.max(1,Math.min(24,(W-L-R)/Math.max(end,1)-2));
  for(const m of d.memory){
    if(!m.paged_mb||m.t<0||m.t>end)continue;
    const cx=x(m.t),top=pp.y(Math.min(m.paged_mb,pMax)),base=pp.y(0),r=Math.min(4,bw/2,base-top);
    root.append(svg('path',{d:`M${cx-bw/2} ${base}V${top+r}Q${cx-bw/2} ${top} ${cx-bw/2+r} ${top}H${cx+bw/2-r}Q${cx+bw/2} ${top} ${cx+bw/2} ${top+r}V${base}Z`,class:'paged'+(m.paged_mb>=d.paged_in_mb?' high':'')}));
  }
  const fp=panels[2];let free='';
  for(const m of d.memory)if(m.t>=0&&m.t<=end&&m.free_mb!=null)free+=(free?'L':'M')+x(m.t).toFixed(1)+' '+fp.y(Math.min(m.free_mb,fMax)).toFixed(1);
  root.append(svg('path',{d:free,class:'series free'}));
  if(!d.memory.length)root.append(svg('text',{x:(L+W-R)/2,y:pp.top+pp.h/2+4,class:'tick','text-anchor':'middle'},'No host memory recorded for this run'));
  root.append(...labels);   // above the marks they label
  // Where HIM stopped the run: our policy's last line.
  if(d.him_reason&&d.end!=null){
    root.append(svg('line',{x1:x(d.end),x2:x(d.end),y1:tp.top-4,y2:fp.top+fp.h,class:'stop'}));
    root.append(svg('text',{x:x(d.end)-6,y:tp.top+12,class:'stop-label','text-anchor':'end'},`HIM stopped: ${d.him_reason}`));
  }
  // Hover: one crosshair through every panel and the readings at that second.
  const cross=svg('line',{y1:tp.top,y2:fp.top+fp.h,class:'crosshair',visibility:'hidden'}),hit=svg('rect',{x:L,y:tp.top,width:W-L-R,height:fp.top+fp.h-tp.top,class:'hit'});
  root.append(cross,hit);box.append(root);
  const tip=node('div','timing-tip');tip.hidden=true;box.append(tip);
  const near=(rows,t,within)=>{let best=null;for(const r of rows)if(Math.abs(r.t-t)<=within&&(!best||Math.abs(r.t-t)<Math.abs(best.t-t)))best=r;return best;};
  hit.addEventListener('pointermove',event=>{
    const rect=root.getBoundingClientRect(),px=(event.clientX-rect.left)*(W/rect.width),t=(px-L)/(W-L-R)*end;
    const s=near(d.seconds,t,1.5);const at=s?s.t:t;
    cross.setAttribute('x1',x(at));cross.setAttribute('x2',x(at));cross.setAttribute('visibility','visible');
    const l=near(d.late,at,0.8),m=near(d.memory,at,0.8);
    const rows=[node('strong','',`${Math.round(at)} s`+(s&&s.sim_time!=null?` · sim ${s.sim_time} s`:''))];
    if(s)for(const k of SERIES){const row=node('span','tip-row');row.append(node('span','legend-key '+k.key),node('span','',k.name),node('b','',fmt(s[k.key],'ms')));rows.push(row);}
    if(l)rows.push(node('span','tip-late',`Late state at ${l.t} s: ${fmt(l.age,'ms')} old, gap ${fmt(l.gap,'ms')}`+(l.missed?`, ${l.missed} missed`:'')));
    if(m)rows.push(node('span','tip-row',`Paged in ${fmt(m.paged_mb,'MB')} · ${fmt(m.free_mb,'MB')} free`));
    tip.replaceChildren(...rows);tip.hidden=false;
    const left=x(at)*(rect.width/W);tip.style.top=`${tp.top*(rect.height/H)}px`;
    tip.style.left=left>rect.width-220?'':`${left+14}px`;tip.style.right=left>rect.width-220?`${rect.width-left+14}px`:'';
  });
  hit.addEventListener('pointerleave',()=>{tip.hidden=true;cross.setAttribute('visibility','hidden');});
}

function drawTable() {
  const d=timingData,box=$('content').querySelector('.timing-table');if(!d||!box)return;
  if(!d.late.length){box.replaceChildren();return;}
  const table=node('table'),head=node('tr');
  for(const h of ['Time','Sim time','Age','Gap','Missed','Paged in that second'])head.append(node('th','',h));
  table.append(node('caption','',`Late states (${d.late_ms} ms or more old)`),head);
  for(const l of d.late){
    const m=d.memory.reduce((b,r)=>Math.abs(r.t-l.t)<0.8&&(!b||Math.abs(r.t-l.t)<Math.abs(b.t-l.t))?r:b,null),row=node('tr');
    for(const v of [`${l.t} s`,l.sim_time!=null?`${l.sim_time} s`:'–',fmt(l.age,'ms'),fmt(l.gap,'ms'),l.missed??'–',m?fmt(m.paged_mb,'MB'):'–'])row.append(node('td','',String(v)));
    table.append(row);
  }
  box.replaceChildren(table);
}
