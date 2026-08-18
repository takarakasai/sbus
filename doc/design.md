# sbus-rs 設計書

`doc/spec.md` のワイヤ仕様を Rust crate として実装するための設計。

---

## 1. クレート構成

```
sbus/
├── Cargo.toml                    # workspace ルート
├── README.md
├── doc/
│   ├── spec.md                   # ワイヤ仕様（実測ベース）
│   └── design.md                 # 本書
└── crates/
    ├── sbus-protocol/            # no_std 純粋プロトコル層
    │   ├── src/{lib,frame,slot,parser}.rs
    │   └── tests/
    │       ├── fixtures/*.bin    # 実機キャプチャ（一次データ）
    │       └── capture.rs        # フィクスチャ回帰テスト
    ├── sbus/                     # serialport 同期ドライバ
    │   └── src/{lib,driver,state,discover,error}.rs
    └── sbus-cli/                 # 動作確認 CLI
        └── src/{main,monitor,render}.rs
```

| クレート | 役割 | `no_std` | I/O |
| --- | --- | --- | --- |
| `sbus-protocol` | フレーム/スロット復号、同期状態機械。alloc・I/O 不使用 | yes | なし |
| `sbus` | Linux serialport ベースの同期受信ドライバ + CH348 ポート探索 | no | あり |
| `sbus-cli` | `monitor` / `dump` / `replay` サブコマンド | no | あり |

分離の理由: ワイヤ解釈はチェックサムを持たない状態機械であり、ここが唯一の
バグ源になりやすい。I/O を含まない層に閉じ込めれば、実機なしでフィクスチャから
バイト単位で完全に再現テストできる（§6）。

---

## 2. `sbus-protocol`

### 2.1 `frame`

```rust
pub const FRAME_LEN: usize = 25;
pub const START: u8 = 0x0F;

/// footer byte (offset 24) classification.
pub enum Footer {
    /// 0x00 — plain S.BUS, no telemetry slots follow.
    Sbus1,
    /// 0x04/0x14/0x24/0x34 — S.BUS2; `group` is 0..=3.
    Sbus2 { group: u8 },
}
impl Footer {
    pub fn from_byte(b: u8) -> Option<Self>;
    pub fn to_byte(self) -> u8;
}

pub struct Frame {
    pub channels: [u16; 16],   // 0..=2047
    pub ch17: bool,
    pub ch18: bool,
    pub frame_lost: bool,
    pub failsafe: bool,
    pub footer: Footer,
}
impl Frame {
    pub fn decode(bytes: &[u8; FRAME_LEN]) -> Result<Frame, FrameError>;
    pub fn channel_us(&self, index: usize) -> Option<u16>;
}

pub fn raw_to_us(raw: u16) -> u16;
```

`Frame` は `Copy`。`decode` が返す誤りは `FrameError::BadStart{found}` /
`FrameError::BadFooter{found}` のみ（チェックサムが無いため他に検出手段が無い）。

生の 25 バイトは `Frame` に持たせない。hexdump 用途では `Event`（§2.3）が
生バイトを別フィールドで運ぶ。`Frame` を制御用途の値型として小さく保つため。

### 2.2 `slot`

```rust
pub const SLOT_LEN: usize = 3;
pub const SLOT_COUNT: usize = 32;

pub const MARKER_RX_BATTERY: u8 = 0xC0;
pub const MARKER_EXTERNAL_VOLTAGE: u8 = 0xC4;
pub const VOLT_LSB_V: f32 = 0.1;

/// Wire ID for telemetry slot `index` (0..=31).
pub fn slot_id(index: u8) -> u8;
/// Inverse of [`slot_id`]; `None` if `id` is not a valid slot ID.
pub fn slot_index(id: u8) -> Option<u8>;

/// Decoded meaning of a 3-byte slot response.
pub enum Telemetry {
    /// slot0 / marker 0xC0 — receiver supply voltage ("Rx-Batt").
    RxBattery { volts: f32 },
    /// slot0 / marker 0xC4 — external voltage input ("Ext-Volt").
    ExternalVoltage { volts: f32 },
    /// Structurally valid slot response we have no verified decode for.
    Unknown { data: [u8; 2] },
}

pub struct SlotResponse {
    pub index: u8,          // slot number 0..=31
    pub telemetry: Telemetry,
}
impl SlotResponse {
    pub fn decode(bytes: &[u8; SLOT_LEN]) -> Option<SlotResponse>;
}
```

設計上の判断:

- **`slot_id` は式で計算する**（`doc/spec.md` §5.1）。32 要素のテーブルを
  手書きすると転記ミスが混入するため。式とテーブルの一致は単体テストで固定する。
- **未知のスロットは捨てず `Telemetry::Unknown` にする。** 25.5 V 超の Ext-Volt で
  marker が変化する可能性が未検証（`doc/spec.md` §7-1）であり、黙って
  誤デコードするより上位でカウントできるほうが安全。
- **電圧は `f32` の V 単位**で返す。`no_std` でも乗算だけなので `libm` は不要。
  生の LSB 値も必要なら `Telemetry::Unknown` 経路と対称にするため、
  `RxBattery`/`ExternalVoltage` にも `raw: u8` を持たせる。

### 2.3 `parser` — 同期状態機械

チェックサムが無く、かつフレームとスロットが時間的に分離して届く
（`doc/spec.md` §3）ため、**バイト単位のプッシュ型パーサ**にする。

```rust
pub enum Event {
    Frame { frame: Frame, raw: [u8; FRAME_LEN] },
    Slot { group: u8, response: SlotResponse, raw: [u8; SLOT_LEN] },
    /// One byte dropped during resynchronisation.
    Desync { byte: u8 },
}

pub struct Parser { /* buf: [u8; FRAME_LEN], len: usize, slot_group: Option<u8> */ }

impl Parser {
    pub fn new() -> Self;
    /// Push one byte. Returns at most one event.
    pub fn push(&mut self, byte: u8) -> Option<Event>;
    pub fn reset(&mut self);
}
```

判定順序（`doc/spec.md` §4 と同一）:

1. `slot_group` が `Some` かつ `len >= 3` かつ `buf[0]` が有効スロット ID
   → スロット応答として 3 バイト消費、`Event::Slot`。
2. `len < 25` → 何も返さず蓄積継続。
3. `buf[0] == START && buf[24]` が有効 footer → 25 バイト消費、`Event::Frame`。
   footer が S.BUS2 なら `slot_group = Some(group)`、S.BUS1 なら `None`。
4. それ以外 → 先頭 1 バイトを捨てて `Event::Desync`。`slot_group = None`。

**内部バッファは 25 バイト固定で足りる。** 不変条件として「各 `push` の
戻り後、バッファに完成した単位は残らない」が成り立つため:

- スロット経路は 3 バイト以上溜まった時点で即消費するので、スロット ID が
  先頭にある状態で 3 バイト以上溜まり続けることはない。
- フレーム経路は 25 バイトに達した時点で必ず消費 or 1 バイト破棄する。
- desync 直後は `slot_group = None` なのでスロット経路が無効化され、
  残り 24 バイトから新たな単位が即座に完成することはない。

よって 1 回の `push` が返すイベントは最大 1 個であり、`Option` で表現できる。
リングバッファも `alloc` も不要。

`Event::Desync` を明示的に返すのは、ゴミバイトとテレメトリの区別が
この実装の要点だからである。呼び出し側が desync 数を監視できないと、
スロット応答の取りこぼしを「正常」と誤認する（Python 版で `skip=249` が
実はテレメトリだった件がこれに該当する）。

---

## 3. `sbus` ドライバ

### 3.1 ポート探索 (`discover`)

固定デバイス名に依存しない（`nm_board/ch348/spec_rev2_0_0_asbuilt.md` §10.7 の要件）。

```rust
pub const CH348_VID: u16 = 0x1A86;
pub const SBUS_UART_INDEX: u16 = 6;

pub struct Ch348Port { pub path: PathBuf, pub uart_index: u16 }

pub fn list_ch348_ports() -> Result<Vec<Ch348Port>>;
pub fn find_sbus_port() -> Result<PathBuf>;
```

手順:

1. `/dev/ttyCH9344USB*` と `/dev/ttyUSB*` を列挙。
2. `/sys/class/tty/<name>/device` から親を最大 8 段辿り、`idVendor` を持つ
   USB デバイスディレクトリを見つけ、VID が `0x1A86` か確認。
3. `ioctl(fd, 0x80025784 /* GETCHIPTYPE */)` ではなく
   `ioctl(fd, 0x80025785 /* GETUARTINDEX */, &mut u16)` で物理 UART 番号を取得。
   これは `_IOC(_IOC_READ, 'W', 0x85, 2)` に一致する（ch9344 ドライバの定義）。
4. `uart_index == 6` のものを SBUS ポートとする。

ioctl が失敗するドライバ（GETUARTINDEX 非対応）では発見順の採番に落とさず
**エラーにする**。Python 版は保険として発見順採番をするが、Rust 側では
誤ポートを黙って開くほうが危険なため方針を変える。CLI の `--port` で明示指定できる。

### 3.2 受信ドライバ (`driver`)

```rust
pub const BAUD: u32 = 100_000;

pub struct Sbus { port: Box<dyn serialport::SerialPort>, parser: Parser, state: State }

impl Sbus {
    pub fn open(path: impl AsRef<Path>) -> Result<Self>;
    pub fn open_auto() -> Result<Self>;          // discover::find_sbus_port()

    /// Read whatever is available and feed the parser, invoking `f` per event.
    pub fn poll(&mut self, f: impl FnMut(&Event)) -> Result<usize>;

    /// Block until the next control frame, or time out.
    pub fn read_frame(&mut self, timeout: Duration) -> Result<Frame>;

    pub fn state(&self) -> &State;
}
```

シリアル設定は 100 000 bps / 8 データビット / 偶数パリティ / 2 ストップビット。
serialport 4.9 は Linux で `termios2` + `BOTHER` により非標準ボーレートを設定できる
（`src/posix/termios.rs`）ので、独自 ioctl は不要。

反転はハードウェア（U15）が行うためソフト反転は入れない。
送信 API は提供しない（受信専用ポート）。

`poll` は 1 回の `read` で得たバイトを 1 バイトずつ `Parser::push` に流す。
read タイムアウトは短く（20 ms）取り、`read_frame` 側で締切を管理する。
`ErrorKind::TimedOut` は空読みとして正常扱いにし、締切超過のみエラーにする。

### 3.3 集約状態 (`state`)

CLI とアプリの双方が欲しい「直近の値 + 累積カウンタ」をまとめる。

```rust
pub struct Counters {
    pub frames: u64,
    pub slots: u64,
    pub unknown_slots: u64,
    pub desync_bytes: u64,
}

pub struct State {
    pub frame: Option<Frame>,
    pub sbus2: bool,
    pub rx_battery_v: Option<f32>,
    pub external_v: Option<f32>,
    pub counters: Counters,
    pub fps: f32,
}
impl State { pub fn apply(&mut self, event: &Event); }
```

`State::apply` は I/O も時刻も触らない純関数的更新（`fps` の更新のみ
`driver` 側が 1 秒窓で行う）。これによりフィクスチャ再生でも同じ集計を検証できる。

---

## 4. `sbus-cli`

`sbus_monitor.py` の機能を包含する。

| サブコマンド | 内容 |
| --- | --- |
| `monitor` | ライブ表示。`--us` / `--plain` / `--raw` / `--seconds` / `--count` |
| `dump` | 生バイト列をファイルに保存（フィクスチャ採取用） |
| `replay` | 保存済みバイト列をオフラインで復号・集計（**実機不要**） |

共通オプション: `--port <PATH>`（省略時は `discover`）。

`replay` を用意する理由は 2 つ。送信機の電源が無い状況でも回帰確認ができること、
および将来の異常キャプチャ（failsafe や 25.5 V 超）を後から解析できること。

表示は Python 版と同じ 3 行ヘッダ + 8 行チャネル表を踏襲する。

---

## 5. エラー方針

```rust
pub enum Error {
    SerialPort(serialport::Error),
    Io(std::io::Error),
    Discovery(String),      // ポート探索に失敗した理由
    Timeout(Duration),
}
```

`thiserror` を使い、既存クレート（`wit-imu`）と同じ形にそろえる。

`sbus-protocol` 側は `thiserror` を使わず（`no_std`）、`FrameError` を
`Debug + Copy` の素の enum にする。

フレーム誤りは「エラー」ではなく `Event::Desync` としてストリームの正常な一部
として扱う。無線リンクでは同期外れが日常的に起きるため、`Result` で
呼び出し側に伝播させるとループが書けなくなる。

---

## 6. テスト戦略

**送信機の電源が無い状態で全テストが通ること**を要件とする。

| 層 | テスト |
| --- | --- |
| `frame` | ビット展開の単体テスト（既知フレーム → 既知チャネル値）、フラグ全ビット、footer 全値 |
| `slot` | `slot_id`/`slot_index` の往復、32 個の一意性、式と `doc/spec.md` §5.1 表の一致、marker 別デコード |
| `parser` | 不変条件（1 push = 最大 1 イベント）、desync 復帰、フレーム跨ぎの分割入力 |
| 統合 | `tests/fixtures/*.bin` の再生。フレーム数・スロット数・**desync 0**・電圧の観測範囲・failsafe 挙動を固定 |

フィクスチャは 2 本ある（`doc/spec.md` §6）。`sbus2_linked_3s.bin` がリンク正常時、
`sbus2_failsafe_8s.bin` が送信機 OFF 時で、後者は failsafe/frame_lost が全フレームで
立ち、チャネルが保持され、テレメトリだけが流れ続ける状態を固定している。

電圧の期待値は「最終値」ではなく **観測された最小/最大の組** で固定する。
Ext-Volt は 1 LSB (0.1 V) 揺れるため、最終値で固定するとキャプチャの
切れ目次第でテストが落ちる。

フィクスチャ回帰テストが、Python 実装との等価性を担保する要になる。
期待値は `doc/spec.md` §6 の実測値をそのまま使う。

またパーサは「1 バイトずつ push」と「チャンク境界をずらして push」の
両方で同一のイベント列を出すことをテストする。実機では read の切れ目が
フレーム中間に落ちるため、ここが壊れると実機だけで再現する不具合になる。

---

## 7. 非目標

- **送信**: CN2 に TX が出ていないため実装しない。S.BUS2 テレメトリハブ機能も対象外。
- **slot1 以降の意味づけ**: 未検証（`doc/spec.md` §7-2,3）。構造だけ通し、
  意味は `Telemetry::Unknown` に留める。
- **非同期 I/O**: 66.5 fps / 1.7 kB/s の単一ポートに async は過剰。同期のみ。
- **`no_std` での I/O**: `sbus-protocol` は I/O を持たないので、組込み側は
  自前のトランスポートから `Parser::push` を呼べばよい。
