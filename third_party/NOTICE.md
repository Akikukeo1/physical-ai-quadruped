# 外部モデル・方策・制御ロジック

大型アセットはGit管理に含めず、`assets/go2-assets.json` の固定URLから
`.assets/go2/` に取得します。各ファイルは使用前にSHA-256を照合します。
取得するのはモデル、メッシュ、ONNX、ライセンス、方策の出典情報のみです。

| 対象 | 出典・固定版 | ライセンス |
| --- | --- | --- |
| Go2 MJCF・16メッシュ | [Unitree公式 unitree_mujoco](https://github.com/unitreerobotics/unitree_mujoco/tree/ae6a8403e272733e9996ef59990880330496177f/unitree_robots/go2) | BSD-3-Clause (`unitree-LICENSE`) |
| 学習済みMoE方策 | [wty-yy/go2_rl_gym](https://github.com/wty-yy/go2_rl_gym/tree/30e74dc507bec7a642a8c98be26081f2c6f0822d) の `deploy/pre_train/go2/go2_moe_cts_high_slope_thre_164k_0.6715.pt` | MIT、継承部分はBSD-3-Clause (`go2_rl_gym-LICENSE`) |
| ONNX変換済み方策 | [syswonder](https://github.com/syswonder/robot-unitree-go2-mujoco/tree/d6e54647541fb7b048db5d9817457ee11d0e2cf9/assets/robots/go2/policy/moe_rough) の `policy.onnx` | 元方策のライセンスを継承 |
| 観測量・速度補償の実装 | [syswonder のネイティブ制御器](https://github.com/syswonder/robot-unitree-go2-mujoco/blob/d6e54647541fb7b048db5d9817457ee11d0e2cf9/sim/native/controller.py) の必要部分を `sim_adapter.py` に移植 | Apache-2.0 (`syswonder-Apache-2.0`) |

syswonderの[NOTICE](https://github.com/syswonder/robot-unitree-go2-mujoco/blob/d6e54647541fb7b048db5d9817457ee11d0e2cf9/NOTICE)では、既存MITコードと新規Apache-2.0コードが区別されています。
制御器の移植ではROS、Robonix、named action、実機通信は使いません。
上位層の地図、距離センサー、模擬設備点検は本プロジェクト側で追加しています。
移植した処理は起動時に方策を読み込み、速度指令からトルクだけを出力する構成に縮小しています。

ONNXのSHA-256は `1d9e76089401a011f38d6e81708f7334618e9faaad078999a1f6d94b07381f1d`。
変換元・変換条件は取得される `policy/moe_rough/UPSTREAM.md` を参照してください。
変換元TorchScriptとの数値一致は変換元プロジェクトの報告であり、ここでは再変換していません。

モデルのXMLはUnitree公式版を直接取得します。実行時にはメッシュパスを絶対パスにし、
物理刻みを0.002秒に設定して、床・照明・録画用カメラを加えます。
初期化時だけ自由関節と関節姿勢を設定し、開始後は `qpos` や外力による移動を行いません。
