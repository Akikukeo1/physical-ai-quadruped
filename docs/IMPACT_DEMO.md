# 立入禁止の追加による探索経路の変更

開発用のseed 305で、実Codex CLIとMuJoCoのGo2を使った1試行。未知配置の青対象を探索し、
途中で「エリア2は危険なので通らないで。元の任務と帰還条件はそのまま」を入力する。
自然言語の解釈はCodex、経路選択と安全検査は観測地図を使うPython、歩行と物理は既存の方策とMuJoCoが担う。
配置と時刻は旧経路を調べて調整した開発条件であり、未見配置での評価ではない。

## 実行結果

| 物理時刻 | 記録された動作 |
| --- | --- |
| 50.0秒 | 指示を受信しゼロ速度を指令。受信時の旧格子経路は新しいエリア2を横切る |
| 50.2秒 | 停止を確認 |
| 53.8秒 | 帰還可能性を確認して制約を適用し、別の探索目標と経路を選択 |
| 65.4秒 | 新しい探索目標の0.35 m以内に到達 |
| 82.3秒 | 実軌跡が旧探索目標の0.3 m以内に到達。禁止領域を避けて同じ地域へ回り込んだ |
| 91.8秒 | 青を距離・色センサーで観測して帰還開始 |
| 113.4秒 | 帰還後の停止完了。出発地点から0.115 m |

元の青探索と帰還のタスクID、期限240秒、帰還地点は保持され、未達成・取消しはない。
受信から適用までの速度指令はすべてゼロ。全トレースで障害物接触・転倒は記録されず、
禁止適用後の0.1秒間隔のロボット中心とエリア2の最短距離は0.614 mだった。
制約後の最初の格子経路の各辺も細分して検査した。探索目標は一度変更されているため、
同じ目標への最短経路同士の比較ではない。制約なしの同時条件の対照走行は今回行っていない。

元ログ: `outputs/impact-dev-2/plan-20261030T061825692778Z.json`  
SHA-256: `30298548caa3d92e3bb4743e9180a9b2b2a0c411f3d194732d9896c07c781323`  
実録画: `outputs/impact-dev-2.mp4`  
SHA-256: `84da37e759d3db73c898e6d5342de5246fc46e7833cba49cc6da2aa42aaff84`  
集計と比較図: `outputs/impact-dev-2/validation.json`, `outputs/impact-dev-2/route-comparison.svg`  
映像素材: `outputs/impact-mission-control-final.mp4`（受信から停止まで、等速、音声なし）

最初の狭い矩形による開発試行は `outputs/impact-dev-1/` と `outputs/impact-dev-1.mp4` に保存した。
旧経路と矩形の交差が境界付近だったため、エリアを広げた上記の試行を採用した。

## 再現

Ubuntu 24.04 / WSL2、認証済みCodex CLI、既存の固定MuJoCo・方策環境を使う。
出力先が存在しない名前を使う。動画と比較図は同名ファイルを上書きする。

```bash
MUJOCO_GL=egl bash scripts/demo.sh --task search --planner codex --seed 305 --headless \
  --field-areas examples/impact-areas.json --correction-script examples/impact-dialogue.json \
  --prompt '青い対象を探し、確認したら出発地点へ帰還して。全体240秒以内' \
  --video outputs/impact-new.mp4 --out-dir outputs/impact-new
```

出力されたログ名を指定して検証と比較図・映像プロジェクトを生成する。
Codexの応答時間や解釈によって結果は変わり、条件を満たさない実行は検証器が失敗する。

```powershell
.venv/Scripts/python.exe scripts/verify_impact_demo.py outputs/impact-new/plan-XXXX.json outputs/impact-new
.venv/Scripts/python.exe scripts/prepare_impact_video.py outputs/impact-new/plan-XXXX.json outputs/impact-new.mp4 outputs/impact-video-new
cd outputs/impact-video-new
npx --yes hyperframes@0.8.220 check
npx --yes hyperframes@0.8.220 snapshot --at '1,8,33,45,62'
npx --yes hyperframes@0.8.220 render --quality delivery --fps 25 --output ../impact-new-control.mp4
```

検証器は旧経路と領域の交差、新経路と領域の距離、禁止後の物理トレース、ゼロ速度待機、
元タスク・期限、センサーの青検出、帰還、接触・姿勢と、実際に両目標付近へ進んだことを検査する。
経路の記録はスキャン時の格子経路で、追従制御はその先読み地点を使う。
比較図と映像の水色線は制約適用後の最初の経路であり、その後の全再計画を示すものではない。

## 表示と限界

映像の地図は受信時までの距離・表面色センサーの観測を固定表示し、位置・向きと軌跡はログから同期する。
ロボット映像にはシミュレータの真の障害物と対象が見えるが、それらを探索器やCodexに渡していない。
目標発見は2本以上のセンサーレイによる観測で、対象への到達や回収ではない。
禁止判定は機体全体ではなく中心に対する離散時刻の検査であり、実機の安全や連続時間の安全を保証しない。
理想的な距離・色センサーと真値オドメトリを使い、カメラ認識やSLAMは行わない。
録画では機体とエリアが小さく、比較図の併用が必要。地図は受信後に増えた観測を表示しない。
観測履歴に基づく再確認の統合は行っていない。独立した既存実験は [CORRECTION_VIDEO.md](CORRECTION_VIDEO.md)。

WSLで物理テストを含む97件が成功。変更ファイルのruff検査・整形検査と `git diff --check` は成功。
Windowsの全体ruff 0.11.6では既存 `motion.py` のUP038が1件残る。Windowsの通常テストは環境の
一時ディレクトリ権限エラーで11件失敗し、WSLの全テスト結果とは区別する。
HyperFrames 0.8.220の検査はエラーなし、コントラスト30件成功、音声なしの警告1件。
実フレームで受信・適用・迂回・観測・帰還時の表示を確認した。
