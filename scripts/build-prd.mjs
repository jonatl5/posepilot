import { readFile, writeFile } from 'node:fs/promises';
const root = new URL('../', import.meta.url);
const spec = JSON.parse(await readFile(new URL('docs/data/prd.json', root), 'utf8'));
const esc = x => String(x).replaceAll('&','&amp;').replaceAll('<','&lt;').replaceAll('>','&gt;').replaceAll('"','&quot;');
const paras = xs => xs.map(x=>'<p>'+esc(x)+'</p>').join('\n');
const list = xs => '<ul>'+xs.map(x=>'<li>'+esc(x)+'</li>').join('')+'</ul>';
const table = (head, rows) => '<div class="table-scroll" role="region" aria-label="'+esc(head.join(' and '))+'" tabindex="0"><table class="evidence-table"><thead><tr>'+head.map(x=>'<th scope="col">'+esc(x)+'</th>').join('')+'</tr></thead><tbody>'+rows.map(row=>'<tr>'+row.map(x=>'<td>'+esc(x)+'</td>').join('')+'</tr>').join('')+'</tbody></table></div>';
const sections = [
 ['overview','Purpose and scope',paras(spec.overview)+table(['Area','Requirement'],spec.scope)],
 ['requirements','Functional requirements',spec.requirements.map(r=>'<article class="requirement" id="'+r.id+'"><h3>'+r.id+' · '+esc(r.area)+'</h3><span class="status-label">'+esc(r.priority)+'</span><p>'+esc(r.requirement)+'</p><p><strong>Current evidence:</strong> '+esc(r.status)+'.</p></article>').join('')],
 ['performance','Operating requirements',table(['Area','Target / constraint'],spec.nonfunctional)],
 ['schema','Observation and action contract',paras(['This is the primary wall-approach contract. Version the complete schema, scaling, history and configuration with every checkpoint.'])+table(['Features','Meaning'],spec.schema)],
 ['sensing','Forward sensing and firmware',list(spec.sensor)],
 ['data','Data and calibration',list(spec.data)],
 ['evidence','Prototype evidence and boundaries',table(['Track','Evidence / limitation'],spec.evidence)+'<div class="inline-links"><a href="./#results">Wall-track results</a><a href="./prototypes.html">Aligned rapid prototypes</a><a href="./evidence/integration/validation.json">Integration validation record</a></div>'],
 ['evaluation','Evaluation and acceptance',list(spec.evaluation)+table(['Gate','Completion evidence required'],spec.gates)],
 ['milestones','Course milestones',table(['Date (2026)','Deliverable'],spec.milestones)],
 ['risks','Risks and open work',list(spec.risks)]
];
const index = await readFile(new URL('docs/index.html',root),'utf8');
const header = index.match(/<header[\s\S]*?<\/header>/)[0].replaceAll('href="#','href="./#').replace('href="./prd.html"','href="./prd.html" aria-current="page"');
const html = '<!doctype html>\n<html lang="en"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width, initial-scale=1"><meta name="theme-color" content="#1435ee"><meta name="description" content="objectPilot requirements: scope, 18 functional requirements, prototype evidence, sensing and hardware acceptance gates."><title>objectPilot — Product requirements</title><link rel="canonical" href="https://jonatl5.github.io/posepilot/prd.html"><link rel="icon" href="./favicon.svg" type="image/svg+xml"><link rel="stylesheet" href="./styles.css?v=prototype-2"><link rel="stylesheet" href="./prototypes.css?v=prd-1"></head><body><a class="skip-link" href="#main">Skip to content</a>'+header+
 '<main id="main"><section class="section-wrap page-intro"><div class="eyebrow">PRODUCT REQUIREMENTS / REPOSITORY EDITION</div><h1>A clear target.<br><span class="blue">A testable path to flight.</span></h1><p>What objectPilot must do, what the prototypes demonstrate, and what still has to pass on the actual aircraft.</p><div class="hero-buttons"><a class="button primary" href="#requirements">Read the requirements</a><a class="button secondary" href="./PRD.md" download>Download PRD Markdown</a></div><p class="release-note">'+esc(spec.edition)+' · '+esc(spec.source_revision)+'.</p></section><div class="section-wrap spec-layout"><nav class="spec-toc" aria-label="Requirements contents">'+sections.map(([id,title])=>'<a href="#'+id+'">'+title+'</a>').join('')+'</nav><div class="spec-content">'+sections.map(([id,title,body])=>'<section id="'+id+'"><h2>'+title+'</h2>'+body+'</section>').join('\n')+
 '<section id="sources"><h2>Sources and maintenance</h2><p>Aligned with the <a href="'+spec.source_url+'">team’s Revision 2 PRD</a> and published October 29 evidence. The Drive document may require team access. This repository edition records the aligned implementation and preserves the original experiment provenance.</p><p>Edit <a href="./data/prd.json">data/prd.json</a> and run <code>npm run build:docs</code> to regenerate both the website and repository PRD.</p></section></div></div></main><footer class="footer section-wrap"><div><a class="footer-brand" href="./">objectPilot</a><span>Requirements grounded in recorded evidence.</span></div><div><a href="./prototypes.html">Explore the aligned prototype ↗</a><span>© 2026 jonatl5 · MIT</span></div></footer></body></html>\n';
const mdTable = (head,rows) => '| '+head.join(' | ')+' |\n| '+head.map(()=>'---').join(' | ')+' |\n'+rows.map(row=>'| '+row.map(x=>String(x).replaceAll('|','\\|')).join(' | ')+' |').join('\n')+'\n';
const bullets = xs => xs.map(x=>'- '+x).join('\n\n');
const md = [
 '# '+spec.title+'\n\n'+spec.edition+'\n\n'+spec.source_revision+'. [Source PRD]('+spec.source_url+').\n\nGenerated from docs/data/prd.json. Run npm run build:docs after editing the source.',
 '## Purpose and scope\n\n'+spec.overview.join('\n\n')+'\n\n'+mdTable(['Area','Requirement'],spec.scope),
 '## Functional requirements\n\n'+spec.requirements.map(r=>'### '+r.id+' · '+r.area+'\n\nPriority: '+r.priority+'.\n\n'+r.requirement+'\n\nCurrent evidence: '+r.status+'.').join('\n\n'),
 '## Operating requirements\n\n'+mdTable(['Area','Target / constraint'],spec.nonfunctional),
 '## Observation and action contract\n\n'+mdTable(['Features','Meaning'],spec.schema),
 '## Forward sensing and firmware\n\n'+bullets(spec.sensor),
 '## Data and calibration\n\n'+bullets(spec.data),
 '## Prototype evidence and boundaries\n\n'+mdTable(['Track','Evidence / limitation'],spec.evidence),
 '## Evaluation and acceptance\n\n'+bullets(spec.evaluation)+'\n\n'+mdTable(['Gate','Completion evidence required'],spec.gates),
 '## Course milestones\n\n'+mdTable(['Date (2026)','Deliverable'],spec.milestones),
 '## Risks and open work\n\n'+bullets(spec.risks),
 '## Evidence links\n\n- [Wall-track diagnostics](https://jonatl5.github.io/posepilot/#results)\n- [Aligned rapid prototypes](https://jonatl5.github.io/posepilot/prototypes.html)\n- [Website PRD](https://jonatl5.github.io/posepilot/prd.html)\n\nThe active implementation follows one 64-value wall-approach contract. Historical diagnostics remain attributed to their original run; no real-flight outcome is claimed.'
].join('\n\n')+'\n';
for (const path of ['PRD.md','docs/PRD.md']) await writeFile(new URL(path,root),md);
await writeFile(new URL('docs/prd.html',root),html);
process.stdout.write('Generated synchronized website and repository PRDs: '+spec.requirements.length+' functional requirements.\n');
