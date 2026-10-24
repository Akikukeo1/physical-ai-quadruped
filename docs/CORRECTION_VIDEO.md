# 判断変更が見える比較プレビュー

以前の録画では、訂正した時点の探索方向と帰還方向が近く、動きから判断の変更を読み取れなかった。
対象色、訂正文、検出履歴、帰還開始の表示もなかった。

ロボットの制御を演出用に変えず、同じシード305を訂正なし／ありで実行し直した。
訂正ありでは探索開始69秒後に「青ではなく緑に変更して」を入力する。
この時点の旧探索目標と出発地点は、ロボットから見てほぼ反対方向にある。
両方とも実際のCodex CLIとMuJoCoを使い、訂正前の物理軌跡が完全一致することを検査する。

14秒のプレビューは物理時刻63〜84秒を1.5倍速で左右に並べる。
訂正受信、停止中、対象の変更、履歴に基づく帰還を実ログの時刻に同期させる。
LLM内部の思考を可視化したものではない。訂正文をCodexが解釈し、観測履歴による帰還判定はPythonが行う。
「発見」は対象の観測であり、対象への到達ではない。全探索の録画も保存する。

## 再現

WSL2で元の録画を作る。出力先が既にある場合は、必要な録画を退避してから実行する。

```bash
MUJOCO_GL=egl bash scripts/demo.sh --task search --planner codex --seed 305 --headless --video outputs/correction-baseline.mp4 --out-dir outputs/correction-comparison/baseline
MUJOCO_GL=egl bash scripts/demo.sh --task search --planner codex --seed 305 --correct-at 69 --correction '青ではなく緑に変更して' --headless --video outputs/correction-turnaround.mp4 --out-dir outputs/correction-comparison/corrected
```

Windows側でプレビューを準備する。動画編集の依存はPython本体から分離し、CLIの版を固定する。

```powershell
npx --yes hyperframes@0.8.192 init outputs/correction-video --non-interactive --skip-skills
.venv/Scripts/python.exe scripts/prepare_correction_video.py
cd outputs/correction-video
npx --yes hyperframes@0.8.192 media-use resolve --adopt --project .
npx --yes hyperframes@0.8.192 check --json
npx --yes hyperframes@0.8.192 snapshot --review
npx --yes hyperframes@0.8.192 preview --background --port 3002
```

再生成時は空のプロジェクトへの初期化を省き、準備スクリプトから実行する。
スクリプトは最新の各ログを読み、完了状態、訂正前の軌跡、元の制約、視界外の緑の履歴、帰還開始順を検査する。
条件を満たさなければ、比較映像の生成を中断する。Codexの応答時間が変わるため、再実行時にも検査が必要。
フォントはHyperframesで扱えるNoto Sans JPを使う。

## 評価上の位置付け

開発用配置で判断の分岐を説明するデモであり、手法の優位性や未知配置への汎化を示す比較実験ではない。
元の動画は保持し、描画の一部で床が欠ける既知の問題も残る。位置・接触・帰還の判定は物理ログで確認する。
動画の短い抜粋だけで帰還完了を証明せず、全体ログと最終誤差も参照する。

## 今回の確認結果

集計は [correction-video-evidence.json](correction-video-evidence.json)。
訂正前の軌跡は一致し、70.0秒に訂正を受信、73.2秒に適用した。
緑の最初の検出は22.0秒で、訂正受信から適用までのスキャンには映っていない。
訂正なしは80.0秒に青を発見して帰還開始、訂正ありは73.2秒に履歴を使って帰還開始。
いずれも全体実行は完了し、最終帰還誤差は順に10.6 cm、9.9 cmだった。

準備スクリプトの自己検査とruff、Hyperframesの実行時・配置・コントラスト・読み取り時間の検査は成功。
最終の動画検査はエラー・警告とも0件、コントラスト45/45件成功。
訂正待ちと分岐後の実フレーム、全体・縮小表示のレビュー画像を確認した。
ロボット制御コードは変更していないため、既存の物理テスト一式は再実行していない。
プレビュー確認後の承認を受け、`outputs/correction-decision-comparison.mp4` に書き出した。
1920×1080、30 fps、14.0秒、420フレーム、H.264、約1.2 MB、音声なし。
書き出し前の再検査も成功し、完成ファイルの全フレームのデコードと、
開始・訂正待ち・分岐後・終盤の画像を確認した。

```powershell
# outputs/correction-video で実行
npx --yes hyperframes@0.8.192 render --quality delivery --fps 30 --output ../correction-decision-comparison.mp4
```
