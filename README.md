# sbus-rs

Futaba S.BUS / S.BUS2 の**受信専用** Rust 実装。
namiashi rev2 (CH348L) の SBUS ポート (UART6 / CN2) を対象とする。

- 25 バイト制御フレーム: 16ch アナログ + ch17/18 + frame_lost + failsafe
- **S.BUS2 テレメトリスロット**: 受信機内蔵の Rx-Batt / Ext-Volt
- 実機キャプチャによる回帰テスト（送信機の電源が無くても全テストが通る）

ワイヤ仕様は [doc/spec.md](doc/spec.md)、設計は [doc/design.md](doc/design.md)。

## クレート

| クレート | 役割 |
| --- | --- |
| [`sbus-protocol`](crates/sbus-protocol) | `no_std` 純粋プロトコル層。フレーム/スロット復号と同期状態機械 |
| [`sbus`](crates/sbus) | serialport ベース同期ドライバ + CH348 ポート探索 |
| [`sbus-cli`](crates/sbus-cli) | `sbus-monitor` バイナリ |

## 使い方

```bash
cargo build --release

# CH348 の tty と物理 UART 番号の対応
./target/release/sbus-monitor ports

# ライブ表示 (Ctrl-C 終了)
./target/release/sbus-monitor monitor
./target/release/sbus-monitor monitor --us            # µs 換算も表示
./target/release/sbus-monitor monitor --seconds 5 --plain
./target/release/sbus-monitor monitor --raw           # フレーム/スロットを hexdump

# 生バイト列の採取と、実機なしでの再生
./target/release/sbus-monitor dump --seconds 5 --out capture.bin
./target/release/sbus-monitor replay capture.bin
```

ポートは `GETUARTINDEX` ioctl で物理 UART 番号 6 を探して開く。
`--port /dev/ttyCH9344USB6` で明示指定も可能。

表示例:

```
SBUS monitor  100000 8E2    66.5 fps   frames=532 slots=133 desync=0
CH17:○  CH18:○   FRAME_LOST:no   FAILSAFE:no
S.BUS2  Rx-Batt:  4.9V  Ext-Volt: 24.1V
--------------------------------------------------------------
  CH 1 1017 ████·········   CH 2 1006 ████·········
  ...
```

## ライブラリとして

```rust
use std::time::Duration;
use sbus::Sbus;

let mut sbus = Sbus::open_auto()?;
loop {
    let frame = sbus.read_frame(Duration::from_millis(100))?;
    if frame.failsafe {
        // Telemetry keeps flowing with the transmitter off, so link health
        // must come from this flag — not from the voltages being present.
        continue;
    }
    println!("CH1={} Ext-Volt={:?}", frame.channels[0], sbus.state().external_v);
}
# Ok::<(), sbus::Error>(())
```

`no_std` 環境では `sbus-protocol` を直接使い、自前のトランスポートから
`Parser::push` にバイトを流す。

## 設計上の要点

- **desync を数える**: S.BUS にチェックサムは無く、S.BUS2 のスロット応答は
  制御フレームの約 2 ms 後に届く。フレームだけを見るパーサはスロット応答を
  ゴミとして捨ててしまう（元の Python 実装で 5 秒あたり 249 バイトがこれだった）。
  `Event::Desync` を明示的に返すのはそのため。
- **推定でデコードしない**: 検証済みの意味付けは slot0 の 2 マーカーのみ。
  それ以外は `Telemetry::Unknown` として生バイトのまま上位に渡す。
- **送信しない**: CN2 に TX が出ていないため、S.BUS2 テレメトリハブにはなれない。

## テスト

```bash
cargo test        # 実機不要
```

`crates/sbus-protocol/tests/fixtures/` の実機キャプチャに対する回帰テストを含む。

## ライセンス

Apache-2.0
