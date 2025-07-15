## What is this?
Primary objective: Handle user requests for the coding agent.
 1. Read and understand the requests below.
 2. Discuss with the user for clarification if needed. 
 3. Move (completed or already-discussed) tasks to docs/TODO.md and clean up request.md.

## User's request
*New requests can be added below. Code agent will processed into TODO.md, and clean up below.*

### Recent Requests (2025-06-27)
✅ **Moved to TODO.md**: Default config memory mode, output organization, penalty function redesign, script naming

- l2距離、ファジーにしてから計算するようにしたい
- コードの変数名やオプション名から期待される挙動と、実際の挙動が食い違う部分を探して、修正する。
- パラメータのハードコードを探し、修正する。
- outputの質を変えるような変更（例えばn_samplesなどの変更）はユーザーに暗黙に行わない。一方で、video非表示などは、最終敵なoutputの質を変化させないので、「クオリティを買えないままなるべく負荷を下げるオプション」みたいなものがあればそれで自動にオフにすればよさそう
- greedyの探索の計算効率は現状で最大ですか？並列計算することにより、たとえば36サンプルの損失を同時に計算することは不可能だろうか？