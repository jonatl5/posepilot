import { readFile, readdir, access } from 'node:fs/promises';
import path from 'node:path';
import { fileURLToPath } from 'node:url';
const root = path.resolve(fileURLToPath(new URL('../docs/', import.meta.url)));
const files = (await readdir(root)).filter(x=>x.endsWith('.html'));
const pages = new Map(await Promise.all(files.map(async name=>[name,await readFile(path.join(root,name),'utf8')])));
let issues=[];
for (const [name,html] of pages) {
 if (!html.includes('<html lang="en">')) issues.push(name+': missing English language declaration');
 if (/[\u3400-\u9fff]/u.test(html)) issues.push(name+': non-English CJK text');
 const ids=[...html.matchAll(/\bid="([^"]+)"/g)].map(m=>m[1]);
 if(new Set(ids).size!==ids.length) issues.push(name+': duplicate IDs');
 for (const m of html.matchAll(/\b(?:href|src)="([^"]+)"/g)) {
  const href=m[1];
  if (/^(https?:|mailto:|data:)/.test(href)) continue;
  const [raw,hash]=href.split('#');
  const clean=raw.split('?')[0];
  let target=path.resolve(root,path.dirname(name),decodeURIComponent(clean||name));
  if(clean.endsWith('/')) target=path.join(target,'index.html');
  if(!target.startsWith(root+path.sep)) {issues.push(name+': link outside site '+href);continue;}
  try {await access(target);} catch {issues.push(name+': missing '+href);continue;}
  if(hash && target.endsWith('.html')) {
   const targetText=await readFile(target,'utf8');
   if(!targetText.includes('id="'+hash+'"')) issues.push(name+': missing anchor '+href);
  }
 }
}
const spec=JSON.parse(await readFile(path.join(root,'data/prd.json'),'utf8'));
const md=await readFile(new URL('../PRD.md',import.meta.url),'utf8');
const publicMd=await readFile(path.join(root,'PRD.md'),'utf8');
if(md!==publicMd) issues.push('Public and repository PRD differ');
if(spec.requirements.length!==18) issues.push('Expected 18 functional requirements');
for(const r of spec.requirements) {
 if(!md.includes('### '+r.id+' · '+r.area)||!pages.get('prd.html').includes('id="'+r.id+'"'))
  issues.push('Missing requirement '+r.id);
}
for(const filename of ['data/prd.json','evidence/integration/validation.json']) {
 const value=await readFile(path.join(root,filename),'utf8');
 if(/[\u3400-\u9fff]/u.test(value)||value.includes('/Users/')) issues.push(filename+': language or local-path leak');
}
if(issues.length) {console.error(issues.join('\n'));process.exitCode=1;}
else console.log('Checked '+pages.size+' English pages, local links, anchors, public evidence and 18 synchronized requirements.');
