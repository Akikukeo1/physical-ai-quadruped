# Phase 1 検証結果（2026-10-24）

Codex CLI が自然言語から生成した JSON を検証し、MuJoCo 内で Go2 が接地したまま
前進・左旋回・待機するところまで確認しました。サンプル計画での右旋回も録画済みです。
Phase 2 の独自タスク・比較実験には着手していません。

## 確認環境と再現

- Windows 11 / WSL2 Ubuntu 24.04、WSLg
- プロジェクト内の Python 3.12.12、uv 0.10.9
- MuJoCo 3.3.7、NumPy 2.2.6、ONNX Runtime 1.23.2、CPU推論
- 認証済み Windows Codex CLI 0.157.0 を WSL から呼び出し
- モデル・方策は `assets/go2-assets.json`、Python依存は `uv.lock` に固定

```bash
MUJOCO_GL=egl bash scripts/demo.sh --planner sample --headless --video outputs/sample.mp4
MUJOCO_GL=egl bash scripts/demo.sh --planner codex --headless --video outputs/codex.mp4 --prompt '2秒前進し、2秒左へ旋回して、2秒待機して'
PAI_SIM_TEST=1 .venv-wsl/bin/python -m unittest discover -s tests -v
```

通常のビューア付き実行も、`--headless --video ...` を外して完了・自動終了を確認しました。
ドライランは `uv run python src/main.py --planner sample --executor dry-run` で確認しました。
環境の準備は [README](../README.md) を参照してください。

## 実測

| 計画 | 結果 |
| --- | --- |
| sample：前進2秒→右旋回2秒→前進2秒→待機2秒 | 前進は約0.39 mと約0.40 m、右旋回は約40度。11秒のMP4を保存 |
| codex：前進2秒→左旋回2秒→待機2秒 | 指示どおりの3動作のJSONを生成。前進約0.39 m、左旋回約41度。9秒のMP4を保存 |
| 2秒の最終停止後 | sample：約0.00071 m/s、codex：約0.00088 m/s。角速度はどちらも絶対値0.001 rad/s未満 |
| 姿勢（10 Hz 記録） | 最小の直立度は sample：0.9990、codex：0.9982。胴体高さの最小値は約0.328 m |

直立度は胴体上方向とワールド上方向の内積で、1が直立です。
全物理ステップで転倒・非有限状態を監視し、10 Hzで状態を記録しています。
動画から複数時刻のフレームを抽出し、接地、位置の変化、旋回、待機を目視確認しました。
描画した動画に、指示生成中の待ち時間は含まれません。

実測の開始・終了状態、使用アセットのハッシュ、ログ名は
[`phase1-validation.json`](phase1-validation.json) に保存しています。
元の詳細ログとMP4はローカルの `outputs/` に残し、Git管理からは除外しています。
フレーム確認用の `outputs/final-contact.png` もローカルに保存しています。

## テスト

- WSL：14テストすべて成功。実物理の前進・後退・左右旋回・停止を含む
- Windows：通常テスト13件成功、実物理テスト1件は明示的にスキップ
- `ruff check src tests`、`ruff format --check src tests`、`git diff --check`：成功

物理テストでは2秒の前後移動が各0.15 m以上、左右旋回が各0.3 rad以上、
停止後の並進速度0.05 m/s未満・角速度0.05 rad/s未満、直立度0.9超、接触ありを検査します。
CLIモックテストと実際のCodexによる上記実行は別の確認です。

## 選定と解消した問題

[MuJoCo Menagerie](https://github.com/google-deepmind/mujoco_menagerie) はモデル集であり、
今回必要な学習済み速度制御器をそれだけでは提供しません。
[quadruped-robotics-stack](https://github.com/darshmenon/quadruped-robotics-stack) も調べましたが、
今回は新規学習やROS/Gazebo環境を必要としない構成を選びました。

[syswonderのネイティブ制御器](https://github.com/syswonder/robot-unitree-go2-mujoco/blob/d6e54647541fb7b048db5d9817457ee11d0e2cf9/sim/native/controller.py)
に公開されている観測形式・速度補償を利用し、同梱のMoE ONNX方策を接続しています。
物理刻み0.002秒、方策推論0.02秒、PDゲイン20/0.5、action scale 0.25、
FL/FR/RL/RRの関節順序と225要素の観測履歴を合わせました。
LLMは動作列だけを決め、バックエンドがトルク制御と物理を担当します。

実装中に次の問題を確認し、解消しました。

- 最初のXML include構成でメッシュ参照先がずれたため、固定版XMLを読み、実行時にメッシュの絶対パスを指定する形に変更
- 候補リポジトリはモデルを「未変更」と説明していたものの、公式との差分に追加センサー等があったため、XMLはUnitree公式版の直接取得に変更。16メッシュも公式固定版と一致することを確認
- WSLのWindows実行用binfmt登録がなく `Exec format error` になったため、WSLの `/init` に実行パスと元のargv[0]を渡す起動方法で解消。OSの設定変更はしていない
- ユーザー設定のアプリ用モデル名が通常のCLIで拒否されたため、認証を保持して `--ignore-user-config` でCLI標準設定を利用。ユーザー設定ファイルは変更していない

外部アセットのライセンス原文と出典は [third_party/NOTICE.md](../third_party/NOTICE.md) にまとめています。

## 残る制限

平坦な床での短時間・低速の検証です。障害物回避、カメラ認識、段差、正確な位置決め、
実機制御、WindowsネイティブのMuJoCo実行は確認していません。
MoE方策の再学習・ONNX再変換は行わず、公開済みアーティファクトを使用しました。
録画は動作確認用で、最終提出用の1〜3分の説明動画は未作成です。
公開リポジトリの作成・pushも行っていません。
