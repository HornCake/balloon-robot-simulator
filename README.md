# 5連結バルーンロボット

MuJoCoで5つのヘリウムバルーンを4つのY軸回転関節で接続し、3次元自由浮遊を計算します。Python 3.12以上。依存管理・実行には **uv** を使用します。

## セットアップと実行

```sh
uv sync --locked
# 画面なしで30秒計算、CSVとグラフを生成
uv run python -m src.main --headless --plot
# macOSのViewer（メインスレッドのためmjpythonを使用）
uv run mjpython -m src.main --realtime
# Linux / WindowsのViewer
uv run python -m src.main --realtime
```

```sh
uv run python -m src.main --headless --neutral --mode fixed --plot --output output/neutral
uv run python -m src.main --headless --no-buoyancy --no-fluid --duration 5 --output output/fall
uv run python -m src.main --headless --mode asymmetric --export-model
uv run pytest
```

Viewerは単独表示・比較表示とも、起動時に画面の作業領域いっぱいへ拡大します。macOSではメニューバーとDockを避けた通常ウィンドウで開きます。従来サイズで開く場合は`--windowed`を追加してください。起動後は通常どおり手動でサイズを変更できます。

単独表示では画面左下に`Horizontal distance`（初期全体重心からの水平移動距離）と`X displacement`（符号付きX変位）をm単位で表示します。水平移動距離は`√(dx²+dy²)`で、上昇分や往復を足し合わせた道のりは含めません。カメラの向きにかかわらず物理座標で測り、Rでリセットすると0に戻ります。停止・待機・計算終了後も表示し、`--headless`でも終了時に同じ2項目を出力します。

単独・比較の両方で、`Simulation time`（物理計算上の経過秒数）、`Running wall time`（開始待ち・一時停止を除いた実時間）、`Playback speed`（計算時間÷実時間）を画面左下に表示します。`Target cycle`は「周期内の経過秒数 / 1周期の秒数」です。Rでこれらもリセットします。単独表示と比較のケース選択中には`Initial shape error`も表示し、全関節0°からの実角度のずれの最大値を度単位で示します。

描画と物理計算は分離し、画面更新の間に複数ステップを計算します。`--realtime`では実時間と計算時間を揃えるように進めますが、計算負荷が高いと追いつかない場合があります。例えば`Playback speed: 0.50x`なら、計算上2秒の動作には実時間で約4秒かかります。

開始前に初期状態を確認したい場合は、次のどちらかを指定できます。単独表示・比較表示の両方で使えます。

```sh
# 停止状態で開き、Spaceを押したら開始
uv run mjpython -m src.main --compare --realtime --start-paused
# 初期状態を5秒間表示してから自動開始
uv run mjpython -m src.main --compare --realtime --start-delay 5
```

待機中は物理計算・シミュレーション時刻を進めず、カメラ操作やFでの表示切替ができます。`--start-delay`はウィンドウが開いてからの実時間[秒]で、Spaceを押すと待機をスキップして開始します。どちらも指定しなければ従来どおり即時開始します。2つのオプションは同時指定できません。Rでリセットすると指定した停止状態または待機時間からやり直します。待機時間は`--duration`に含めません。`--headless`では`--start-delay`を使えますが、Space入力ができないため`--start-paused`は使用できません。

`--realtime`なしでは高速計算します。指定時間（デフォルト30秒）に達すると計算を止め、Viewerは最後の状態を表示したまま残ります。終了後もカメラ操作やRでのリセットができ、ウィンドウを閉じるとプログラムが終了します。`--plot`のグラフ生成もウィンドウを閉じた後に行います。ViewerはSpaceで一時停止・再開、Rでリセット、1で俯瞰、2で側面、3で追従。Fで空気抵抗用の形状の表示をオン・オフできます（初期状態はオフ）。表示中は空力形状を半透明のオレンジ色にし、外観メッシュを薄くします。デフォルトの表面抵抗方式ではOBJと同じ形状、従来方式では楕円体が表示されます。これは形状の表示切替であり、空気抵抗の有効・無効は変更しません。マウスでカメラを操作できます。リセット後もCSVに追記し、時刻が0に戻る行が新しい試行の開始です。実行ごとに異なる`--output`を指定すると結果を保存できます。同じ出力先では上書きします。

## 登録済みの関節動作と同時比較

balloon-studioの保存データから13パターンの**実際の角度式**を登録しています。追加の接続なしで呼び出せます。

```sh
# パターンIDと名前の一覧
uv run python -m src.main --list-patterns
# 1パターンを再生
uv run mjpython -m src.main --pattern air-01 --start-paused --realtime
# 異なる動作のロボットを並べて同時再生
uv run mjpython -m src.main --compare-patterns air-01 air-04 air-10 --start-paused --realtime
# 計算結果をCSVと比較グラフへ保存
uv run python -m src.main --compare-patterns air-01 air-04 air-10 --headless --plot --output output/motions
# YAMLで比較対象を登録する場合
uv run mjpython -m src.main --config config/motion_comparison.example.yaml --compare --realtime
```

| ID | 動作 |
| --- | --- |
| `air-01` | 尾側ほど大きい進行波 |
| `air-02` | 尾側を強調した緩急波 |
| `air-03` | S字を経由して戻す |
| `air-04` | 尾側2関節だけで振る |
| `air-05` | 包絡付き尾側進行波 |
| `air-06` | 左右交互の緩急波 |
| `air-07` | 前後逆位相の尾振り |
| `air-08` | 逆順に戻す緩急波 |
| `air-09` | S字回復の非対称波 |
| `air-10` | 小さく構えて強く振る波 |
| `slow-sweep` | ゆっくり構えて、振り抜く |
| `paddle` | 全関節の同時パドル |
| `wave-01` | Wave Motion 01 |

保存式における1周期は「全関節の角度波形が同じ状態で繰り返すまで」の時間です。元の周期は`config/motion_patterns.yaml`の`period`に保存し、式と照合しています。`air-01`〜`air-10`（`air-06`を除く）と`slow-sweep`は8秒、`air-06`は16秒、`paddle`は5秒、`wave-01`は2秒です。

```sh
uv run mjpython -m src.main --compare-patterns air-06 paddle wave-01 --period 8 --start-paused --realtime
uv run mjpython -m src.main --pattern air-07 --period 2 --neutral --realtime
```

再生時の共通周期は`control.period`で指定し、既定設定は2秒です。`--period 8`なら全ケースを8秒周期にします。元の周期に対して時間軸だけを伸縮し、位相差と、曲げる・戻す・待つ時間の比率を保ちます。例えば`air-06`は元の式で8秒後に符号が反転し、16秒後に元へ戻るため、共通周期2秒では1秒で逆側の動作に移り、2秒で全体が繰り返します。`paddle`の起動時3秒待ちは保存式から削除済みで、元周期・共通周期のどちらでも直ちに動き始めます。開始前の待機には`--start-delay`または`--start-paused`を使ってください。従来の`sine`・`traveling`・`asymmetric`にも`control.period`が適用されます。

真っすぐに戻すための包絡補正は行いません。保存式が開始時に曲がった形を指定していれば、その目標角をそのまま使います。`air-05`・`air-07`など保存式自体に含まれる包絡はそのままです。最後に`motion_scale`の倍率と±90度の目標角制限を適用します。

共通周期は物理計算上の目標角の周期で、実関節には慣性・負荷・トルク上限による追従遅れがあります。実角度は`Initial shape error`とCSVで確認できます。周期境界で物理状態を強制リセットすることはありません。

取り込み時の周期に戻す場合はYAMLで`control.period: null`にします。この場合、登録パターンは保存式の元の時間軸、従来モードは`control.frequency`を使います。過去の完全な保存設定で`period`がないものも元の時間軸を保ちます。過去の設定を使って比較周期を揃える場合は`--period 2`などを明示してください。これまでの周期が異なる条件での距離比較結果は、共通周期の結果とは区別してください。

`--compare-patterns`は指定した順にロボットを表示します。同じIDを複数回指定することもできます。各ロボットは同じモデル・物理条件で独立して計算し、開始・一時停止・リセットを共有します。表示・カメラ操作は下記の比較モードと共通です。`--fluid-model ellipsoid`などで全ケースの物理条件を変更できます。`--pattern`は`--mode`と同時指定できず、`--compare-patterns`は他の比較指定・`--pattern`・`--mode`と同時指定できません。

登録先は`config/motion_patterns.yaml`です。各IDに`name`と`expressions`を追加すれば新しい動作を呼び出せます。単独起動のYAMLでは`control: {pattern: air-01}`で指定できます。登録せずに式を書く場合は、既定設定を継承して次のように指定します。

```yaml
extends: default.yaml
control:
  mode: expressions
  expression_period: 8  # 下記の元の式の1周期。共通周期への時間変換に使用
  expressions:
    - A = 20
    - T = 8
    - x(n,t) = A*n/4*sin(2*pi*t/T - (n-1)*pi/3)
```

`x(n,t)`は関節番号`n=1..4`、秒単位の時刻`t`から度単位の目標角を返します。式は上から定義し、四則演算・剰余・`sin`・`cos`・`floor`・`min`・`max`・`pi`と、先に定義した定数・関数を使えます。最終目標角には従来どおり±90度制限がかかります。登録パターンでは`amplitude_deg`・`frequency`・`phase_step_deg`など従来モード専用の項目は使わず、式中の定数を編集します。

2026-11-20取得時点のStudioの式・プロジェクトID・revisionを保存しています。Studio側の変更は自動同期しません。説明文に60度と書かれた`slow-sweep`も保存式は20度、`paddle`は保存式に従い−45度です。`paddle`の起動時3秒待機は取得後にローカルで削除しています。取り込むのは関節角の指令で、バルーン寸法・質量・浮力や基部の選択はこのシミュレータの設定を使います。登録ファイルの`period`と`start_delay`は元の式の周期・起動待ちを表し、読み込み時に`control.expression_period`・`control.expression_start_delay`として保存します。独自の式にも共通周期を適用する場合は元の1周期を`expression_period`に指定してください。元の時間軸のまま使う場合は`period: null`にします。

出力のケース別`config.yaml`には使用した式と取得元、`comparison.yaml`にはパターンIDを保存します。保存された設定は式を内包するため、後から登録内容を変更しても同じ式で再実行できます。保存設定で別の登録パターンを選び直す場合は`--pattern`を指定してください。

振幅だけを変更する場合は`--motion-scale`を使います。全関節の目標角を指定倍率にし、周期・位相・緩急のタイミングを保ちます。例えば最大20度の`air-02`を最大60度にするには次を実行します。

```sh
uv run mjpython -m src.main --neutral --pattern air-02 --motion-scale 3 --start-paused --realtime
# wave-01の倍率1/2とair-07の20/60度を、中性浮力で同時比較
uv run mjpython -m src.main --config config/amplitude_comparison.example.yaml --compare --start-paused --realtime
```

YAMLでは`control.motion_scale`で指定できます。既定値は1、0なら全関節0度、負の値は指定できません。従来の制御モードにも適用でき、倍率をかけた後の目標角は±90度に制限します。`--compare-patterns`と併用すると全ケースに同じ倍率を適用します。異なる倍率を同時比較する場合は`comparison.cases`内の`control`に`pattern`と`motion_scale`をそれぞれ指定してください。大きな目標角でも実角度が同じだけ増えるとは限らず、トルク上限と負荷によって追従が遅れます。

## 比較モード

同じOBJの楕円体方式とメッシュ表面方式をY軸方向に並べる場合：

```sh
uv run mjpython -m src.main --compare ellipsoid mesh_surface --realtime
```

比較対象は起動オプションまたはYAMLで任意の数を選択できます。同じ方式を複数指定することもできます。`--compare`だけならYAMLの`comparison.cases`を使い、ケース一覧がなければ楕円体とメッシュ表面の2ケースになります。通常起動は単独表示のままです。

モデルごとにOBJ・向き・寸法・抵抗係数を変える場合は`config/comparison.example.yaml`をコピーして編集します。付属の例は「楕円体＋balloon_87＋カプセル」の3ケースで、そのまま実行できます。

```sh
uv run mjpython -m src.main --config config/comparison.example.yaml --compare --realtime
# GUIなしで比較し、個別グラフと重ね合わせグラフを生成
uv run python -m src.main --config config/comparison.example.yaml --compare --headless --plot --output output/compare
```

例えば「メッシュ1＋メッシュ2＋メッシュ3」だけを使う設定は次の形です。`mesh_file`を実際のファイルに置き換えてください。

```yaml
extends: default.yaml  # 継承元はこのYAMLからの相対パス
comparison:
  cases:
    - name: Mesh1
      fluid_model: mesh_surface
      balloon:
        mesh_file: assets/mesh1.obj
        mesh_fit_dimensions: true
    - name: Mesh2
      fluid_model: mesh_surface
      balloon:
        mesh_file: assets/mesh2.obj
        mesh_rotation_deg: [0, 0, 90]
        mesh_fit_dimensions: true
    - name: Mesh3
      fluid_model: mesh_surface
      balloon:
        mesh_file: assets/mesh3.obj
        mesh_fit_dimensions: true
      physics:
        surface_drag_coefficient: 0.8
```

各ケースは基準設定を継承し、指定した項目だけを上書きします。`name`が表示ラベルです。ケースに`mesh_file`を指定した場合は、明示的に`fluid_model: ellipsoid`を選ばない限りメッシュ表面方式を使います。指定したOBJの各三角形が抵抗計算の対象になります。ケースごとに`balloon`、`physics`、`joint`、`control`などを変更でき、同じ名前でも出力番号で区別します。

比較ケースはY軸方向のレーンに並べ、X方向の表示オフセットは0にします。異なるモデルのY寸法に合わせてレーン間隔を確保します。初期表示は以前と同じ斜めからの透視表示です。各ケースの`dx`は、そのケースの初期全体重心からのX変位[m]です。X=0の基準線と+X方向の矢印も表示します。F表示は最初からオンです。

Space・R・カメラ切替は全ケース共通です。1は斜めからの透視表示（`--camera overview`）、2は真横からの表示（`--camera side`、仰角0度）、3は中心を追従する斜め表示（`--camera follow`）、4は真上からの平行投影（`--camera top`）です。1・2ではカメラ位置・ズームを毎フレーム上書きせず、マウス操作で調整できます。3でもユーザーが調整した角度とズームを保ちます。4はマウスのXY移動・ズームを保ち、高度だけ追従します。Rで計算とカメラを初期状態に戻します。

真横ではY方向のレーンが重なるため、比較中の2はケース1だけを表示します。Tabまたは`]`で次のケース、`[`で前のケースを表示します。`0`で全ケースの斜め表示に戻ります。`--focus-case 2`のように起動時に表示するケースを指定することもできます。非表示のケースも計算・記録は継続します。

真横で全ケースを同時表示する場合は`--camera side --side-all`を指定します。ケースを色分けして半透明で重ね、ラベルには対応する色名も表示します。投影は既定で平行投影となり、Y方向の奥行きによる見かけの大きさの違いを除きます。物理座標や表示レーンをずらさず、同じX位置で比較できます。Aキーで全ケース／1ケースを切り替えられます。`--projection perspective`を追加すると透視表示も選べます。

画面上の+Xは以前と同じ右向きが既定です。左向きにする場合は`--x-direction left`を指定します。この設定はカメラの見る側だけを変え、物理座標やCSVの符号は変えません。実際の推進方向は下記の関節間位相差で変更します。各カメラの投影方式は`--projection perspective`または`--projection orthographic`で明示指定できます。投影方式を切り替えると表示範囲も適切な単位に戻します。

```sh
# 以前と同じ斜めの透視表示で、停止状態から開始
uv run mjpython -m src.main --compare --camera overview --start-paused --realtime
# 比較ケース2を真横から確認
uv run mjpython -m src.main --compare --camera side --focus-case 2 --start-paused --realtime
# 真横から全ケースを色分けして同時比較
uv run mjpython -m src.main --compare --camera side --side-all --start-paused --realtime
# 真上表示、+Xが画面右になる向き
uv run mjpython -m src.main --compare --camera top --x-direction right --start-paused --realtime
```

物理計算はケースごとに独立しており、横並びのオフセットは表示にだけ適用します。CSV座標に表示オフセットは入りません。全ケースの時間刻み・実行時間は揃える必要があります。計算負荷はケース数とメッシュの面数に応じて増えます。

別々の設定ファイルを直接並べることもできます。各YAMLは完全な設定、または`extends`で基準設定を継承する設定にします。`--duration`などの起動オプションは全ケースに適用します。

```sh
uv run mjpython -m src.main --compare-config config/model1.yaml config/model2.yaml config/model3.yaml --realtime
```

比較結果は`output/run/01_名前/trajectory.csv`、`config.yaml`のようにケース別に保存し、ルートの`comparison.yaml`に対応表を書き出します。`--plot`では各ケースの`summary.png`と全ケースを重ねた`comparison.png`を生成します。比較グラフは重心X変位・基部の符号付きX速度・重心高さ・基部の速さの4項目です。リセット後のX変位は各試行の開始位置から計算します。比較中の`--export-model`は`models/comparison/01/`などに実際の計算用モデルを個別出力します。画面上でケースを追加・削除する機能はなく、ケース一覧を変更して再起動します。

## 設定と物理モデル

`config/default.yaml`を編集するか`--config`で別のYAMLを指定します。SI単位を使用し、`*_deg`のみ度です。制御は`fixed`、`sine`、`traveling`、`asymmetric`から選択。非対称波の`bend_fraction`は曲げ動作が占める周期比率です。目標角は±90度に制限されます。実角度の関節制限はMuJoCoの柔らかい拘束なので、負荷によって微小な超過が生じることがあります。

進行波の関節間位相差は`control.phase_step_deg`です。既定は−45度です。0.5 Hzの進行波の30秒条件では、全体重心X変位は楕円体方式で約+2.00 m、メッシュ表面方式で約+0.73 mでした。瞬間的な基部速度には関節運動による往復が含まれるため、推進は周期をまたいだ全体重心の変位で確認してください。モデル・風・姿勢・係数を変更した場合の方向を保証する設定ではありません。

`--phase-step-deg -45`で既定の進行波、`--phase-step-deg 45`で以前の逆向きの進行波を指定できます。比較ケース内でも`control: {phase_step_deg: ...}`で個別に設定できます。これは実際の関節目標を変更し、カメラや座標の符号を反転する処理ではありません。

- 外観：`assets/balloon_87.obj`を使用。元のOBJの−X/Z/Y軸をシミュレーションのX/Y/Z軸に対応させ、バウンディングボックスの中心を原点に移し、各軸を450×650×300 mmに合わせたコピーです。以前の向きからZ軸まわりに90度回転し、長手方向をY軸、並び方向をX軸、高さ方向をZ軸にしています。隣接端部の隙間10 mm、中心間隔460 mm。関節は隙間の中央。`mesh_file: null`では楕円断面カプセルメッシュを自動生成します。
- 質量：膜26 gとヘリウム11.62 gを各バルーンに設定し、関節22 gを接続先の4剛体にそれぞれ1回だけ加算。合計276.1 g。
- 慣性：均質楕円体と関節円柱を合成し、平行軸の定理で部品質量の位置を反映。`src/model.py`の`composite`とバルーン慣性生成を分離してあり、シェル・実測モデルは未実装です。
- 浮力：外観と独立した70 Lから各0.8274735 Nを計算。ローカル浮力中心をワールド座標へ変換し、その位置に外力を適用。ヘリウム重量は剛体質量に含まれるため再度差し引きません。
- 中性浮力：合計145.65 gの追加バラストを5体に均等配分。各バルーン下部（中心から150 mm下）に29.13 gずつ配置します。位置は中心基準の`ballast_position`で変更でき、点質量として扱い、重心・慣性に反映します。総浮力と総重量の一致は姿勢の静的釣り合いを保証しません。関節部品による重心偏りで回転は起こり得ます。
- 空力：デフォルトは`physics.fluid_model: mesh_surface`。OBJの各三角形の面積・外向き法線・重心位置から、表面圧力による抵抗とモーメントを計算します。局所速度には並進、回転、関節運動、風を含めます。二次抵抗は独自に適用し、MuJoCo側の流体密度を0にして二重加算を防ぎます。粘性抵抗のみMuJoCoの楕円体近似を併用します。`--no-fluid`ですべての空力を無効化します。浮力用の空気密度は保持します。
- 高さ0 mに市松模様の地面を表示します。表示専用で衝突・支持拘束はなく、落下時は地面を通過します。自由基部＋4回転関節で計10自由度。SG90の詳細モデルやCFD、膜変形、空力干渉は含みません。

流体係数は[MuJoCo公式の流体モデル](https://mujoco.readthedocs.io/en/stable/computation/fluid.html)の順序（鈍頭抗力、細長物体抗力、角抗力、Kutta揚力、Magnus揚力）です。推進量の実機一致を保証するモデルではありません。

### OBJ形状を使う空気抵抗とローポリ化

MuJoCo標準の空力はメッシュでも楕円体近似なので、表面の形を反映するために`src/aerodynamics.py`で表面抵抗を追加しています。各面の外向き法線をn、空気に対する面中心の速度をv、面積をAとすると、面の力は`F = -0.5 × 空気密度 × C × A × max(v・n, 0)² × n`です。空気側へ進む面だけに力を加え、面中心の位置からトルクを求めます。Cは`physics.surface_drag_coefficient`（初期値1.0）で調整します。閉じていて面の向きが揃った三角形メッシュが必要です。

これは準定常の圧力抵抗近似です。後流、剥離、面やバルーン同士の風の遮蔽、CFDによる圧力分布は計算しません。Kutta・Magnus揚力モデルもこの方式では使いません。細かい面が増えても実際の空気抵抗の精度を保証せず、抗力係数は実測で調整する必要があります。

現在のOBJは1,230頂点・2,456三角形で、まずはこのまま使用します。描画と抵抗計算には同じメッシュを使い、面のデータは初期化時に準備して5体で共有します。軽量化が必要になったら別の簡略化メッシュを用意し、各方向の投影面積と同じ運動条件での力・トルクがどの程度変わるか比較してください。

`physics.fluid_model: ellipsoid`にすると、`dimensions`に基づく従来のMuJoCo楕円体方式に戻ります。`fluid_coefficients`はこの従来方式で使い、`surface_drag_coefficient`は表面抵抗方式で使います。

## 出力と構成

`output/run/trajectory.csv`に全ステップを記録し、`config.yaml`に使用条件を保存します。`--plot`で`summary.png`を生成します。CSVには時刻、基部位置・姿勢（w,x,y,z）、ワールド座標の基部線速度・角速度、各剛体重心、関節目標・実角度（rad）・角速度（rad/s）・トルク（N m）、各浮力（N）、全体重心が含まれます。状態と力は同一時刻で評価します。

`src/model.py`が設定に応じたMJCFとOBJを生成し、通常の計算では一時ディレクトリから読み込みます。`assets/`のOBJは入力アセットとしてGitで管理します。`--export-model`は現在の設定のXMLと隣接OBJを`models/`に出力します。`models/`は再生成できるためGitの管理対象から除外しています。

### 手持ちのOBJモデルを使う

バルーン1個分のOBJを、例えば`assets/my_balloon.obj`に置き、YAMLの`balloon`に次を設定します。5個のバルーンに同じ外観を使います。

```yaml
balloon:
  # 既存のdimensions、volume、film_massなどは残す
  mesh_file: assets/my_balloon.obj
  mesh_scale: [0.001, 0.001, 0.001]  # mm単位のOBJの場合。mなら[1, 1, 1]
  mesh_rotation_deg: [0, 0, 90]  # 必要な場合のみ。固定軸X→Y→Zの順
  mesh_center: true
  mesh_fit_dimensions: true  # 回転後の各軸寸法をdimensionsに合わせる
```

相対パスはプロジェクトルート基準で、絶対パスも指定できます。三角形の面を持つOBJを書き出してください。取り込み時にスケール→固定軸X/Y/Z回転→中心合わせ→寸法合わせの順に適用します。`mesh_fit_dimensions: true`では各軸を個別に拡大縮小します。元の縦横比を保つ場合はfalseにして`mesh_scale`を揃え、最終的な寸法に`dimensions`も合わせてください。配置の基準はXが並び方向、Yが長手、Zが高さです。MTLや外部テクスチャは読み込まず、現在のバルーン色で表示します。元のOBJは変更せず、実行用ディレクトリに変換済みの`custom_balloon.obj`を作ります。`mesh_file: null`で自動生成に戻ります。自動生成される`models/balloon.obj`を直接差し替えても、次回の出力で上書きされます。

単独起動・比較起動とも、コマンドからも別のOBJを指定できます。

```sh
uv run mjpython -m src.main --mesh /path/to/another.obj --mesh-rotation 0 0 90 --fit-mesh --realtime
uv run mjpython -m src.main --mesh /path/to/another.obj --fit-mesh --compare ellipsoid mesh_surface --realtime
```

OBJの変更は外観と表面抵抗方式の空力形状に適用します。質量・慣性・浮力はOBJから自動計算しません。`dimensions`は慣性・接続間隔と従来方式の空力形状、`volume`は浮力、`film_mass`は膜質量を決めるので、実物に合わせて別途設定してください。ロボット全体のOBJは、そのままでは関節ごとに動きません。バルーン単位に分割して使う必要があります。

`simulation.py`は物理ステップ、`buoyancy.py`は浮力、`aerodynamics.py`はOBJ表面抵抗、`controller.py`は波形、`logger.py`はCSV、`visualization.py`はViewerとグラフを担当します。浮力中心はsite group 4、空力形状はgeom group 3です。デバッグ表示を追加する場合はこれらと`data.xipos`を使用できます。

検証は自由落下、実浮力、総質量、中性浮力での重心位置、浮力中心によるモーメント、内部駆動時の重心保存、方向別流体抗力、4関節の独立追従、全波形の目標制限を対象にしています。

標準Euler積分では内部運動に伴う重心の数値ドリフトが生じます。隙間10 mm・長手方向Y軸・位相差−45度での無外力の進行波試験2秒では、ステップ幅2 msで約8.13 mm、1 msで約4.06 mmです。重心保存テストは0.5 msと0.25 msで誤差の収束を確認しています。微小な推進量を評価する際はステップ幅を変えた収束確認が必要です。

## 開発とGitHubでの管理

クローン後はリポジトリのルートで`uv sync --locked`を実行します。`pyproject.toml`と`uv.lock`を一緒に管理し、依存関係を変更した場合はロックファイルも更新してください。

```sh
uv run --locked pytest -q
uv run --locked python -m src.main --headless --duration 0.01 --plot --output output/smoke
uv run --locked python -m src.main --headless --compare --duration 0.01 --plot --output output/comparison
```

GitHub ActionsはpushとPull Requestで、Linux・Python 3.12のテストと、単独・比較モードのCSV／グラフ生成を実行します。Viewerの操作テストはウィンドウを模擬するため画面接続を必要としません。実際のウィンドウ表示やmacOSの拡大処理は、Viewerを起動して別途確認してください。

`src/`、`tests/`、`config/`、`assets/`、依存定義、READMEと`.github/`を管理します。仮想環境、キャッシュ、`output/`、生成された`models/`、`.env`は除外します。再利用する実験スクリプトは`output/`の外に置き、共有したい実験結果は必要なものを選んで別途保存してください。現在の入力OBJは小さいため、Git LFSは必要ありません。

公開リポジトリにする場合のライセンスは未設定です。配布条件と入力OBJの公開可否を確認してから追加してください。
