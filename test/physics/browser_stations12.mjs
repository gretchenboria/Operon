import { fileURLToPath } from "node:url";
import { dirname, resolve } from "node:path";
const APP = process.env.OPERON_URL || "http://127.0.0.1:8841";
const REPO = resolve(dirname(fileURLToPath(import.meta.url)), "..", "..");
import WebSocket from "ws";
import { spawn } from "node:child_process";
import { rmSync } from "node:fs";
const PROF="/tmp/cr-prof-"+process.pid;
const CDP=9361;
const chrome=spawn("/Applications/Google Chrome.app/Contents/MacOS/Google Chrome",
 ["--headless=new","--disable-gpu","--no-sandbox","--no-first-run","--use-gl=angle","--use-angle=swiftshader",
  "--enable-unsafe-swiftshader",`--remote-debugging-port=${CDP}`,"--remote-allow-origins=*",
  "--window-size=1500,950",`--user-data-dir=${PROF}`,"about:blank"],{stdio:"ignore"});
process.on("exit",()=>{try{chrome.kill("SIGKILL")}catch{};try{rmSync(PROF,{recursive:true,force:true})}catch{}});
const sleep=ms=>new Promise(r=>setTimeout(r,ms));
let t=null;
for(let i=0;i<60;i++){try{const l=await (await fetch(`http://127.0.0.1:${CDP}/json/list`)).json();t=l.find(x=>x.type==="page");if(t?.webSocketDebuggerUrl)break;}catch{} await sleep(250);}
const ws=new WebSocket(t.webSocketDebuggerUrl,{perMessageDeflate:false});
await new Promise(r=>ws.once("open",r));
let id=0;const pend=new Map();const errs=[];
ws.on("message",raw=>{const m=JSON.parse(raw.toString());
 if(m.id&&pend.has(m.id)){pend.get(m.id)(m);pend.delete(m.id);return;}
 if(m.method==="Runtime.exceptionThrown")errs.push(m.params.exceptionDetails.exception?.description||m.params.exceptionDetails.text);});
const send=(me,pa={})=>new Promise(r=>{const i=++id;pend.set(i,m=>r(m.result));ws.send(JSON.stringify({id:i,method:me,params:pa}));});
const ev=async e=>{const r=await send("Runtime.evaluate",{expression:e,returnByValue:true,awaitPromise:true});return r?.result?.value;};
await send("Runtime.enable");await send("Page.enable");
await send("Page.navigate",{url:`${APP}/teleop-studio.html`});
for(let i=0;i<90;i++){await sleep(400);const b=await ev(`document.getElementById("phys-badge")?.textContent||""`);if(b&&!/booting/i.test(b))break;}
let fail=0;const ok=(c,m)=>{console.log(`${c?"  PASS":"  FAIL"}  ${m}`);if(!c)fail++;};
const verdict=()=>ev(`document.getElementById("verdict-6")?.textContent||""`);

console.log("[Station 1] charger dock — analytic cell logic must still complete");
await ev(`document.querySelector('#st-bar button[data-st="1"]').click()`);
await sleep(1200);
await ev(`document.getElementById("btn-auto").click()`);
await sleep(70000);
let v=await verdict(); console.log(`  verdict: "${v}"`);
ok(/Slotted|Complete/i.test(v),"Station 1 completes — cell physically seated in the dock");

console.log("\n[Station 2] valve handwheel — physics-owned angle");
await ev(`document.querySelector('#st-bar button[data-st="2"]').click()`);
await sleep(1200);
const before=await ev(`document.getElementById("verdict-6")?.textContent||""`);
console.log(`  before: "${before}"`);
await ev(`document.getElementById("btn-auto").click()`);
for(let i=0;i<26;i++){ await sleep(900); const s=await verdict(); console.log(`    t=${((i+1)*0.9).toFixed(1)}s  ${s}`); }
v=await verdict();
ok(/Valve Rotated 90|Complete/i.test(v),`Station 2 reaches 90 deg under real stem friction — got "${v}"`);

const shot=await send("Page.captureScreenshot",{format:"png"});
if(shot?.data){const fs=await import("node:fs");fs.writeFileSync("/tmp/operon_st2.png",Buffer.from(shot.data,"base64"));console.log("  screenshot /tmp/operon_st2.png");}
const real=errs.filter(e=>!/favicon|ReadPixels|GL Driver/i.test(e));
console.log(`  page errors: ${real.length}`); real.slice(0,6).forEach(e=>console.log("   ! "+e.split("\n")[0]));
ok(real.length===0,"no page errors across stations 1 and 2");
console.log(`\n${fail===0?"STATION 1+2 INTEGRATION PASSED":fail+" FAILED"}`);
ws.close();process.exit(fail===0?0:1);
