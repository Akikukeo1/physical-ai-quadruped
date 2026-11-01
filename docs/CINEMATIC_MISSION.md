# MISSION REVISION: 映像と実験の対応

提出用映像は `outputs/mission-cinematic/mission-revision-final.mp4`。制約更新と観測履歴を使う訂正は、それぞれ独立した seed 305 の開発試行であり、同じミッションで同時に成立したとは示していない。

## 実験と結果

- 実験 A: `outputs/impact-dev-2/plan-20261030T061825692778Z.json`。50.0 秒で立入禁止指示を受信し、50.2 秒に停止、53.8 秒に area2 を適用した。受信時の旧経路は area2 と交差し、適用直後の新経路は避けている。青を 91.8 秒に観測し、113.4 秒に帰還、誤差 0.115 m。記録上、障害物接触と転倒はない。制約はロボット中心に適用され、機体全体の非侵入を保証しない。
- 実験 B: `outputs/task-revision-dev/reobserve/plan-20261024T201811354659Z.json`。56.0 秒に緑の再確認を指示し、62.4 秒に履歴の `obs-47-green` を参照する訂正を受理。74.4 秒の新しいセンサー観測で緑の再確認を達成し、青を 117.4 秒に観測し、帰還誤差 0.119 m。元の青探索、帰還義務と180秒の期限を維持した。記録上、障害物接触と転倒はない。A の立入制約はこの実験にはない。
- 複合指示の新規試行: `outputs/mission-combined-trial/plan-20261031T051827054530Z.json`。指示時には緑の観測履歴がなく、観測 ID の確認が必要になった。訂正は適用されず、青は未発見のまま帰還した。複合シナリオの成功例ではない。

## 映像とログの対応

映像冒頭 0–4 秒は A の物理時刻 50–54 秒のダイジェストで、その後に時刻を戻すことを画面に明示している。

| 映像時刻 | ログの物理時刻 | 内容 |
| --- | --- | --- |
| 0:04–0:17 | A 49–62 秒、1倍 | 制約の受信 50.0 秒、適用 53.8 秒 |
| 0:17–0:30.5 | A 62–89 秒、2倍 | 迂回の歩行 |
| 0:30.5–0:42.7 | A 89–113.4 秒、2倍 | 青の観測、帰還 |
| 0:42.7–0:48 | 別実験の区切り | 実験 A の制約を引き継がないと明記 |
| 0:48–0:54.67 | B 45–55 秒、1.5倍 | 緑の過去の観測地点 |
| 0:54.67–1:05 | B 55–65.33 秒、1倍 | 訂正受信 56.0 秒、受理 62.4 秒 |
| 1:05–1:15.67 | B 65.33–76 秒、1倍 | 方向変更、新たな緑の観測 74.4 秒 |
| 1:15.67–1:36.87 | B 76–118.4 秒、2倍 | 青探索の継続、青の観測 117.4 秒 |
| 1:36.87–1:46.87 | B 118.4–138.4 秒、2倍 | 帰還 |
| 1:46.87–1:58.87 | 結果 | 独立した実験結果と限界 |

倍速と物理時刻は画面にも表示する。地図の観測済み領域は、その時点までの距離・表面色センサーの記録を `ObservedMap` に再入力したもの。ロボット位置・向きはシミュレータ真値。紫点は緑の真の座標ではなく、観測したロボットの位置である。

## 映像の生成と限界

新規録画はログの速度指令を同じ MuJoCo、Go2、学習済み ONNX 歩行方策で再実行した。各指令終了時に記録された位置・ヨー角と比較し、最大差は A・B とも 0。再生はログに記録された指令の再実行であり、新たな LLM の意思決定ではない。前方カメラは位置とヨー角に追従する映像用の安定化された仮想視点で、探索器のカメラ入力ではない。

地面の帯は記録された受信時の旧経路と適用直後の新経路を表示するレンダリング専用の形状であり、その後の現在経路を保証しない。幅 0.9 m はモデルの半径 0.45 m の可視化で、安全な通過幅を示すものではない。描画形状は物理モデル・センサー・衝突判定に追加しない。観測範囲、未探索部分、実軌跡と真値を混同しないこと。

自然言語の解釈は既存の Codex CLI、行動の検証・探索・制約・帰還は既存の実行器、関節制御は既存の学習済み方策が担当する。実機安全性や未知環境全般への汎化は検証していない。学習済みモデルと依存物のライセンスは `third_party/NOTICE.md` を参照。効果音は生成スクリプトで合成した短い正弦波で、BGM とナレーションはない。

## 再現

WSL の既存環境と入力ログを使う。録画は既存ファイルを上書きしない新しい名前へ出力する。

```bash
MUJOCO_GL=egl .venv-wsl/bin/python outputs/mission-cinematic/replay_fpv.py outputs/impact-dev-2/plan-20261030T061825692778Z.json outputs/mission-cinematic/restriction-detail-new.mp4 --start 49 --end 68
MUJOCO_GL=egl .venv-wsl/bin/python outputs/mission-cinematic/replay_fpv.py outputs/task-revision-dev/reobserve/plan-20261024T201811354659Z.json outputs/mission-cinematic/reobserve-fpv-new.mp4 --overhead outputs/mission-cinematic/reobserve-overhead-new.mp4 --start 1 --end 138.4
PAI_SIM_TEST=1 .venv-wsl/bin/python -m unittest tests.test_search tests.test_field_areas tests.test_agent_planner
```

`outputs/mission-cinematic/prepare_final.py` は上記ログと既定の録画を検査し、地図データと HyperFrames プロジェクトを生成する。既存の `final-project` がある場合は上書きを拒否する。再生成時は既存成果を退避してから実行する。完成済みプロジェクトからの再レンダリング例:

```powershell
cd outputs/mission-cinematic/final-project
npx --yes hyperframes@0.8.223 check
npx --yes hyperframes@0.8.223 render --quality delivery --fps 30 --output ../mission-revision-rerender.mp4
```

検証ログと映像素材は `outputs/mission-cinematic/` 内。録画メタデータは `restriction-detail-fpv.json` と `reobserve-fpv.json`。完全な A 前方録画の最初のフレームは物理時刻 1.04 秒である。
