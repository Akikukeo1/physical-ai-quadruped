# Physical AI — Quadruped Motion Planner

自然言語の指示を Codex CLI が JSON の動作列に変換し、MuJoCo 内の Unitree Go2 を動かします。
学習済みの MoE 歩行方策を ONNX Runtime で実行するため、方策の学習や実機は不要です。

**Phase 1 は Ubuntu 24.04 / WSL2 で実行確認済みです。**
サンプル計画と実際の Codex 計画で、接地した Go2 の前進・旋回・停止を録画しました。
後退と左右旋回も物理テストで検証しています。
[検証記録](docs/PHASE1_RESULTS.md)と[出典・ライセンス](third_party/NOTICE.md)を参照してください。
既知の地図で障害物を避け、指定地点を順に訪れるタスクも追加しました。
移動してきた障害物で経路が塞がれた場合に、静止して経路を選び直すデモも実行確認済みです。
課題の独自性の検討、多場面での比較実験、最終提出物はこれからです。

## 通路封鎖からの再計画

```bash
bash scripts/demo.sh --task replanning --planner codex
```

最初は北側から青→緑へ進みます。シミュレーション開始4秒後から6秒後にかけて、
オレンジの箱が北側の通路へ移動します。残りの経路が塞がれたことを検出すると、
Go2を静止させ、現在位置・観測した障害物・未訪問の目的地をCodexへ渡します。
代替経路も同じ安全検査を通したうえで実行し、青→緑の訪問順を維持します。

```bash
MUJOCO_GL=egl bash scripts/demo.sh --task replanning --planner codex --headless --video outputs/replanning-codex.mp4
```

同じ固定の初期計画から3方式を比べる場合は、次を個別に実行します。
`stop` は封鎖を検出して未到達で終了するため、終了コード1が期待されます。

```bash
bash scripts/demo.sh --task replanning --planner sample --replanner stop --headless
bash scripts/demo.sh --task replanning --planner sample --replanner rules --headless
bash scripts/demo.sh --task replanning --planner sample --replanner codex --headless
```

今回の1場面ではルール方式とCodex方式は同じ経路で到達しました。Codexの幾何経路探索の優位性は示していません。
[比較結果と制限](docs/REPLANNING_RESULTS.md)を参照してください。

再計画は最大2回、Codexの応答待ちは各60秒までです。失敗・不正経路・訪問順変更は停止で終了し、
別方式への自動フォールバックは行いません。判断中はシミュレーション時間を一時停止します。
実時間で連続して動く障害物への対応や、実機の制御周期を保証する構成ではありません。

## 高度タスク：障害物を避けて地点を順に訪問

```bash
bash scripts/demo.sh --task navigation --planner codex --prompt '赤い障害物の北側を回って青い地点へ行き、その後で緑の地点へ移動して'
```

Codex は既知の地図から `north_gate → blue → green` のような経路を JSON で選びます。
プログラムは MuJoCo の実測位置を0.2秒ごとに読み、各地点へ向く速度指令に直します。
観測はシミュレータの座標を使っています。色の名前は地図に付けたラベルで、カメラによる色認識ではありません。

赤い箱は衝突判定のある障害物、青と緑の円は目的地、黄色の円は出発地点です。
地図上の北はワールド座標の +y 方向です。最大8地点、移動90秒、各地点35秒で打ち切ります。
経路全体について、障害物をロボット半径0.45 mと余裕0.10 mぶん広げて交差を検査します。
実行中は接近・接触・転倒を監視します。異常時は再計画せず、トルクをゼロにしてシミュレーションを中断します。

地点追従は前進0.25 m/s、旋回0.6 rad/s以下で動き、停止後の最終位置誤差が0.12 m以下なら完了です。
LLM の不正な経路を固定経路へ自動置換する処理はありません。安全でない計画は拒否します。
固定計画との動作確認には `--planner sample` を使えます。
このタスクは実測状態を必要とするため、`--executor dry-run` は対応しません。

```bash
MUJOCO_GL=egl bash scripts/demo.sh --task navigation --planner codex --headless --video outputs/navigation-codex.mp4 --prompt '赤い障害物の北側を回って青い地点へ行き、その後で緑の地点へ移動して'
```

実行ログには経路、訪問順、各地点の到達誤差、停止後の位置、障害物との距離・接触を記録します。
[実測結果と制限](docs/NAVIGATION_RESULTS.md)も参照してください。

## MuJoCo デモ（WSL2）

WSLg が利用できる Ubuntu 24.04 で、リポジトリのルートから実行します。
確認環境は uv 0.10.9、Python 3.12.12、MuJoCo 3.3.7、ONNX Runtime 1.23.2 です。

WSL に uv がない場合は、固定版をプロジェクト内に用意できます。

```bash
mkdir -p .tools
curl -fL https://github.com/astral-sh/uv/releases/download/0.10.9/uv-x86_64-unknown-linux-gnu.tar.gz -o .tools/uv-linux.tar.gz
tar -xzf .tools/uv-linux.tar.gz -C .tools
```

次の一行で環境の準備から実行まで行います。

```bash
bash scripts/demo.sh --planner sample
```

Python は `.python-wsl/`、仮想環境は `.venv-wsl/` に作成します。
初回は依存ライブラリと約34 MBのモデル・方策を取得するため、ネット接続が必要です。
アセットはコミットと SHA-256 で固定し、以後も実行前に照合します。大型バイナリは Git 管理しません。
ビューアはデモ完了時に閉じます。シミュレーション速度は描画環境によって実時間より遅くなる場合があります。

自然言語で指示する場合は、Codex CLI をインストールして `codex login` を済ませます。
WSL の `codex` を優先し、ない場合は PATH 上の Windows 版 `codex.exe` を利用します。
Windows の認証情報をコピーする必要はありません。今回の実行では Windows 版 0.157.0 を使いました。

```bash
bash scripts/demo.sh --planner codex --prompt '2秒前進し、2秒左へ旋回して、2秒待機して'
```

ビューアを開かずに録画する場合は、EGL を使います。

```bash
MUJOCO_GL=egl bash scripts/demo.sh --planner sample --headless --video outputs/sample.mp4
MUJOCO_GL=egl bash scripts/demo.sh --planner codex --headless --video outputs/codex.mp4 --prompt '2秒前進し、2秒左へ旋回して、2秒待機して'
```

720×480、25 fps の MP4 と、計画・実測状態を含む `outputs/plan-*.json` が残ります。
動画ファイルは指定先を上書きします。実行ログのファイル名は時刻ごとに変わります。

スクリプトを使わない場合は、WSL 内で次のように実行できます。
Windows 用 `.venv` との混用を避けるため、環境変数を指定してください。

```bash
export UV_PROJECT_ENVIRONMENT="$PWD/.venv-wsl"
export UV_PYTHON_INSTALL_DIR="$PWD/.python-wsl"
export UV_CACHE_DIR="$PWD/.uv-cache"
uv run --locked --python 3.12.12 --extra sim python src/main.py --planner sample --executor mujoco
```

## 計画と制御

```text
自然言語 → Codex CLI → JSON Schema + Python 検証 → 動作別の速度指令
                                                  ↓
                                     MoE 方策 50 Hz → PD制御 → MuJoCo 500 Hz
```

通常の動作・地点タスクではCodexを計画生成時に一度だけ呼びます。
再計画タスクでは封鎖検出後に最大2回追加で呼び、シミュレーションの各フレームでは呼びません。
シミュレータへ渡すのは検証済みの計画と制限付きの速度指令です。
Python やシェルコードを LLM に生成・実行させません。
計画生成には一時ディレクトリを使い、CLI のシェル・Apps・プラグインを無効化します。
`--ignore-user-config` により CLI 標準のモデル設定を使います。既存の認証は利用しますが、ユーザー設定ファイルは変更しません。

| 動作 | 指令 |
| --- | --- |
| `forward` | 前進 0.18 m/s |
| `backward` | 後退 0.12 m/s |
| `turn_left` / `turn_right` | 左右旋回 0.35 rad/s |
| `wait` | 速度指令ゼロ |

通常の `--task motion` の計画は **12動作以下、合計15秒以下、各0.2〜2.0秒** に制限します。
アダプタ側でも前後0.3 m/s、横0.2 m/s、旋回0.6 rad/s以下に制限します。
これらは指令値であり、実速度の厳密な上限や到達位置を保証するものではありません。
方策の内部入力は公開制御器の速度補償に従い、それぞれ0.6、0.35、1.0以下に制限します。
トルクはモデルのモーター上限で制限し、姿勢の崩れ・低すぎる胴体位置・非有限状態を検出したら実行を中断します。

初期姿勢の設定後は、関節トルクと接地によって動かします。ロボットを吊るしたり、座標を書き換えて移動させたりはしません。
計画の前に1秒の姿勢安定化、後に2秒の速度ゼロ指令を入れるため、サンプル録画は11秒です。
ログには指令ごとの開始・終了状態と、10 Hz の位置・向き・速度・接触数を保存します。
途中で失敗した場合も計画と取得済みの実行ログを残します。

## ドライランとテスト

次のコマンドは追加ライブラリなしで動作し、PowerShell でも検証済みです。

```bash
uv run python src/main.py --planner sample --executor dry-run
uv run python -m unittest discover -s tests -v
```

実際の物理テストは、デモ実行で環境を用意した後に WSL で実行します。

```bash
PAI_SIM_TEST=1 .venv-wsl/bin/python -m unittest discover -s tests -v
```

通常テストは物理テスト5件をスキップします。物理テストでは前進・後退距離、左右旋回角、
停止時の並進・角速度、直立姿勢、接地に加え、障害物の迂回、封鎖後の停止・再計画、順番どおりの到達を検査します。
Codex の利用はモックテストと実行記録を区別しています。

## 現在の制限と残りの課題

検証範囲は平坦な床、既知地図と1種類のスクリプトで動く障害物です。
カメラ認識・任意の動的環境・段差・実機制御は実装していません。
Windows ネイティブの MuJoCo 実行は未検証です。方策の ONNX 再変換や再学習も行っていません。
通常の秒数指定の動作は位置や角度を保証しません。地点タスクは許容誤差を検査しますが、停止後も微小な姿勢調整があります。

次の段階では独自のコミュニケーション課題を定義し、固定計画との比較実験を行います。
最終提出には1〜3分の説明動画、公開リポジトリ、PDFレポートが必要です。
GitHub への公開や push はまだ行っていません。
