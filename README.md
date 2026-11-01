# Physical AI — Quadruped Motion Planner

自然言語の指示を Codex CLI が JSON の動作列に変換し、MuJoCo 内の Unitree Go2 を動かします。
学習済みの MoE 歩行方策を ONNX Runtime で実行するため、方策の学習や実機は不要です。

冷蔵倉庫の温度異常を調べる[設備調査モード](docs/INVESTIGATION.md)を追加しました。
Codexが合成の設備マニュアルと現場の点検履歴を参照し、仮説・次の点検先・証拠付き報告を更新します。
移動は既存の距離センサー、観測地図、経路計画、MuJoCo Go2と歩行方策を使います。
設備の状態は近傍で停止したときだけ取得する模擬インターフェースで、画像認識や実機計測ではありません。
温度・風速を個別に選び、機体の位置・向きと可動プローブの先端幾何を合わせる
[運動学的プローブによる計測](docs/PROBE_MEASUREMENTS.md)も追加しました。
`--probe` 使用時は機体への質量・慣性を追加せず、温度・風速の取得に距離・方向・静止・遮蔽条件を要求します。
設備の3D表示と、全景・Go2追従の同期録画は[実演・収録手順](docs/WAREHOUSE_DEMO.md)を参照してください。
実行時の判断・仮説・観測値を映像に表示し、フレームとログをシミュレーション時刻で対応付けます。

```bash
# 準備済みのWSL環境、リポジトリルートで実行
.venv-wsl/bin/python scripts/investigate.py --scene A --headless
# 運動学的プローブで温度・風速を測り、送風不足を調査
.venv-wsl/bin/python scripts/investigate.py --scene A --probe --development-fault airflow_fault --headless
```

**Phase 1 は Ubuntu 24.04 / WSL2 で実行確認済みです。**
サンプル計画と実際の Codex 計画で、接地した Go2 の前進・旋回・停止を録画しました。
後退と左右旋回も物理テストで検証しています。
[検証記録](docs/PHASE1_RESULTS.md)と[出典・ライセンス](third_party/NOTICE.md)を参照してください。
既知の地図で障害物を避け、指定地点を順に訪れるタスクも追加しました。
移動してきた障害物で経路が塞がれた場合に、静止して経路を選び直すデモも実行確認済みです。
さらに、地図や対象座標を渡さず、センサー観測から対象を探して帰還するタスクを追加しました。
探索途中の訂正では、対象変更に加え、過去の対象の再確認、探索の追加・順序変更・取消しを扱います。
実Codexで「緑を再確認 → 青の探索を再開 → 帰還」を実行確認しました。
観測履歴の有無を[別配置で比較](docs/CORRECTION_EVALUATION.md)した初回8試行も保存しています。
訂正の機会があったのは4配置中1配置で、一般的な汎化成功率はまだ示していません。
さらに[追加12配置・24試行](docs/CORRECTION_EVALUATION_EXPANDED.md)を実行しましたが、訂正の機会は1配置に限られました。
最終提出用の説明動画・レポートはこれからです。

フィールドに長方形エリアを配置し、実行途中の指示でエリアごとに通行可否を変更する実験も追加しました。
配置方法、MuJoCoでの表示、実試行の結果と制限は [フィールドエリア](docs/FIELD_AREAS.md) を参照してください。
LLMに次の探索候補の選択と、タスク・エリアの一括訂正を任せる[実験モード](docs/AGENT_SEARCH.md)も用意しています。

## 観測履歴に基づくタスクの訂正

```bash
# 緑が未観測の状態で訂正：緑の探索を続けて帰還
bash scripts/demo.sh --task search --planner codex --seed 0 --correct-at 8 --correction 'やっぱり緑を探して' --headless
# 過去に緑を観測したが、現在は見えない状態で訂正：履歴を使って帰還
bash scripts/demo.sh --task search --planner codex --seed 305 --correct-at 55 --correction '青ではなく緑に変更して' --headless
# 履歴を参照して緑を再確認し、元の青探索を残す
bash scripts/demo.sh --task search --planner codex --seed 305 --correct-at 55 --correction '青の探索は続けて。ただし、さっき見た緑をもう一度確認してからにして。最後は戻ってきて' --headless
# 青を取り消し、すぐ帰還する
bash scripts/demo.sh --task search --planner codex --seed 0 --correct-at 8 --correction '青の探索は中止して、今すぐ出発地点へ戻って' --headless
```

`--correct-at` は探索開始からのシミュレーション秒です。単発の訂正を再現します。
複数回の訂正・確認応答には `--correction-script examples/dialogue-short-answer.json` またはライブ入力ファイルを使います。
帰還開始前に最大12メッセージを受け付けます。曖昧な指示は質問し、ゼロ速度で回答を待ちます。
Codexへ現在位置、観測履歴の要約、現在見える色、未達成タスク、残り時間を渡します。
返答は `search / reobserve / return_home` を組み合わせた、最大8件の順序付きタスクと取消し記録のJSONです。
元のタスクは同じIDと内容で保持し、取消しには対象IDと入力文からの引用を要求します。
観測地図・検出履歴・帰還条件・元の締切は保持します。隠れた対象座標や正解地図は渡しません。
`search` は過去の検出でも完了できます。`reobserve` は開始後の新しい観測が必須で、必要なら観測時のロボット位置へ戻ります。
どちらも対象への到達・回収ではなく、距離・色センサーによる確認です。
応答待ちはゼロ速度を指令し、物理と観測を進めます。応答後も停止を確認してから新しいタスク列へ切り替えます。
未対応の訂正、不正出力、モデルの失敗は実行失敗として停止し、古い依頼を勝手に再開しません。
確認質問に回答が来ない場合は、元の帰還余裕を使い切る前に `not_found` として帰還します。

実行中の入力にはUTF-8ファイルも使えます。WSLの別ターミナルから訂正文を書いてください。
既存の文字列は開始直後に読み込まれるため、起動前に空にします。入力できる時間を確保するにはビューア付きで実行します。

```bash
mkdir -p outputs
: > outputs/correction.txt
bash scripts/demo.sh --task search --planner codex --seed 305 --correction-file outputs/correction.txt
```

別ターミナルで、探索中に次を実行します。

```bash
printf '%s\n' 'やっぱり緑を探して。最後に戻る条件はそのままで' > outputs/correction.new
mv outputs/correction.new outputs/correction.txt
```

訂正文は最大500文字、ライブ入力ファイルは4096バイト以下です。新しい文へ置き換えると次の入力として受け付けます。
同じ文をもう一度送る場合は、毎回異なるIDを付けた `{"id":"turn-2","message":"緑です"}` をUTF-8で書き込みます。
同じIDや同じ通常テキストの再読み込みでは再実行しません。全文を書いた別ファイルから原子的に置き換えてください。
`sample` は初期の青探索と固定の緑への訂正を使う輸送確認用で、自然言語を解釈しません。
[MVPの試行結果・再現手順](docs/CORRECTION_MVP.md)に、開発用の実Codex試行と制限を記録しています。
[拡張したタスク訂正の検証記録](docs/TASK_REVISION_RESULTS.md)に、プロトコルと実行結果をまとめています。
[複数回の訂正と確認対話](docs/DIALOGUE_RESULTS.md)に、成功例・失敗と修正・無回答時の帰還を記録しています。
[判断変更を見せる比較プレビュー](docs/CORRECTION_VIDEO.md)では、同じ探索の訂正なし／ありを並べます。

## 未知配置の対象探索と帰還

```bash
bash scripts/demo.sh --task search --planner codex --seed 301 --prompt '青い対象を探して出発地点へ戻って。全体180秒以内'
bash scripts/demo.sh --task search --planner codex --seed 302 --prompt '青ではなく緑を探して、見つけたら帰還して'
```

障害物と対象の配置はシードから生成します。**シード・正解地図・対象座標・固定の迂回地点を探索側やCodexへ渡しません。**
Codexが生成するのは `target`（blue/green）、`seconds`（60〜240秒）、`on_found: return_home` と意図のJSONです。
探索側は観測から0.1 m格子の地図を作り、未探索領域が見えそうな場所を選び、既知の安全な範囲で経路を更新します。
見つけた後は地図上の経路で出発地点へ戻り、停止後の誤差0.15 m以内を検査します。

観測は **理想的な360度・最大1.8 mの距離／表面色センサー**です。MuJoCoのレイキャストを2度刻みで行い、
最初に当たった面の距離と色だけを1 Hzで受け取ります。壁の裏や範囲外の対象は検出しません。
カメラ画像の物体認識ではなく、可視面の色をシミュレータから取得する簡易センサーです。
自己位置と向きはシミュレータの正解値を使います。SLAMや自己位置推定の検証ではありません。

発見は指定色が2本以上のレイに映ることです。対象への接近・回収・接触を意味しません。
帰還時間を残して探索を打ち切り、見つからなければ `not_found` と記録します。この結果は対象の不在を証明しません。
灰色の箱・壁と青／緑の箱には衝突判定があり、黄色の円が出発地点です。
安全監視と評価ログは正解の障害物情報を使いますが、それを探索の経路判断へ返しません。

```bash
# 固定JSONでの動作確認：青を探す。sampleではpromptを解釈しない
bash scripts/demo.sh --task search --planner sample --seed 0 --headless
# 青の対象が存在しない場面
bash scripts/demo.sh --task search --planner sample --seed 0 --missing-target --headless
# 録画
MUJOCO_GL=egl bash scripts/demo.sh --task search --planner codex --seed 0 --headless --video outputs/search.mp4
# 事前に宣言した未見配置14条件を全て実行。Codexは色ごとに1回だけ呼び、計画を固定
.venv-wsl/bin/python scripts/evaluate_search.py --planner codex --out-dir outputs/search-evaluation
```

評価条件は [search-evaluation.json](docs/search-evaluation.json)、結果と限界は [SEARCH_RESULTS.md](docs/SEARCH_RESULTS.md) に記録します。
この評価が扱うのは同じ生成方式内の未見配置への適応です。未知の物体種類・地形・センサーへの汎化や、LLMの経路探索の優位性を示すものではありません。

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

通常の動作・地点・探索タスクではCodexを計画生成時に一度だけ呼びます。
探索の訂正入力を使う場合は、各入力で追加の解釈を行います。質問への回答も含め、最大12回です。
再計画タスクでは封鎖検出後に最大2回追加で呼び、シミュレーションの各フレームでは呼びません。
シミュレータへ渡すのは検証済みの計画と制限付きの速度指令です。
Python やシェルコードを LLM に生成・実行させません。
計画生成には一時ディレクトリを使い、CLI のシェル・Apps・プラグインを無効化します。
`--ignore-user-config` でユーザー設定を読み込まず、モデルを `gpt-6-luna`、推論強度を `xhigh` に明示固定します。既存の認証は利用しますが、ユーザー設定ファイルは変更しません。過去の実験はモデル未指定で実施したため、この設定での結果とは区別してください。

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

通常テストは物理テスト9件をスキップします。物理テストでは前進・後退距離、左右旋回角、
停止時の並進・角速度、直立姿勢、接地に加え、障害物の迂回、封鎖後の停止・再計画、順番どおりの到達を検査します。
探索ではセンサーの遮蔽・入力情報の制限と、未知配置での発見・帰還も検査します。
訂正の物理テストでは、緑が未観測の場合と、過去に観測した緑が現在は見えない場合の両方を検査します。
履歴の緑を新たに観測し直してから、元の青探索と帰還を完遂するタスク列も検査します。
Codex の利用はモックテストと実行記録を区別しています。

## 現在の制限と残りの課題

検証範囲は平坦な床、既知地図、1種類のスクリプトで動く障害物、および静的な未知配置の色付き対象探索です。
カメラ認識・任意の動的環境・段差・実機制御は実装していません。
Windows ネイティブの MuJoCo 実行は未検証です。方策の ONNX 再変換や再学習も行っていません。
通常の秒数指定の動作は位置や角度を保証しません。地点タスクは許容誤差を検査しますが、停止後も微小な姿勢調整があります。

次の段階では言語理解・探索・制御の寄与を分けた比較実験を行います。
最終提出には1〜3分の説明動画、公開リポジトリ、PDFレポートが必要です。
GitHub への公開や push はまだ行っていません。
