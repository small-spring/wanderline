環境構築はuvを使う

wanderlineは絵を描くエージェントです。

- input: 
    - canvas_t: wanderlineが線を引くカンバス。wanderlineはここに一筆書きを行う。
    - motif_t: wanderlineが見ている画像のこと。
- output_t: 角度
    - 最終的にはロボットアームの動きとかにしたい。
    - 角度が出力で、その方向に1cm線を引くとかにしたい
    - canvas_tをcanvas_next?に更新します


