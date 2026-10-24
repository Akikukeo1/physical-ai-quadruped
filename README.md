# Physical AI — Quadruped Motion Planner

Codex CLI の自然言語出力を、制限付きの動作列に変換する最小構成です。

> **現在の状態**：Codexとの接続・JSON検証・ドライラン・テストは実装済み。実際の四脚ロボットを動かすMuJoCoバックエンドは**未実装**です。これだけでは最終課題の提出要件を満たしません。

## すぐに動かす（WSL2 / PowerShell共通）

Python 3.11〜3.13が必要です（`.python-version` は3.12）。`uv` を使う場合も追加ライブラリ不要です。Codex CLIの認証・実行は手元のPCで確認してください。

```bash
uv run python src/main.py --planner sample --executor dry-run
uv run python -m unittest discover -s tests -v
```

Codex CLI がインストール・ログイン済みなら、実際のLLMを使用できます。

```bash
uv run python src/main.py --planner codex --executor dry-run --prompt '右方向を案内する動きで、四脚ロボットを動かして'
```

標準出力に動作命令が表示され、`outputs/` に計画とログが保存されます。計画はCodexから直接JSON Schemaに従って出力され、Python側でも独立に検査します。

## 動作の語彙

- `forward`: 前進
- `backward`: 後退
- `turn_left` / `turn_right`: 左右旋回
- `wait`: 停止・待機

現時点では **合計12ステップ以下、15秒以下、各0.2〜2.0秒** に制限しています。速度の初期値はソース内の仮設定であり、実際の速度制限は採用する制御器に合わせて見直してください。

## MuJoCoとの接続

`src/pai_motion/sim_adapter.py` が差し替えポイントです。実際に動作する学習済み制御器か逆運動学 (IK) 制御器を接続するまでは実行できません。

Codexに渡す実装タスクは [`docs/NEXT_CODEX_TASK.md`](docs/NEXT_CODEX_TASK.md) を参照してください。

## 提出前に必要なこと

- MuJoCo内で **物理的に動く** 四脚ロボットを実行する
- Codexの計画で実動作を変えられることを示す
- 新規タスクの定義・先行例との比較を明示する
- 1〜3分の動画、公開GitHubリポジトリ、PDFレポートを準備する
- 元となるロボットモデルと歩行制御器の出典・ライセンスを記載する

LLMを開発支援に使うことと、ロボットがタスク実行時にAIを使うことは区別します。後者は `--planner codex` で実現します。
