#!/usr/bin/env python3
"""
DJI 動画（Osmo Action 6 等 / dvtm_ac206.proto 形式）から
時刻・GPS・IMU(姿勢クォータニオン/角速度) などのテレメトリを抽出する。

DJI の新しめのカメラは、旧来のサイドカー .SRT ではなく MP4 内の
timed-metadata トラック（handler="CAM meta", codec="djmd"）に
protobuf 形式でフレーム毎のテレメトリを埋め込む。本スクリプトは
外部ツール(ffmpeg/exiftool)無しで純Pythonだけで解析する。

使い方:
    python extract_dji_telemetry.py Source/DJI_xxx.MP4
    python extract_dji_telemetry.py Source/DJI_xxx.MP4 -o out.csv --json out.json

出力: 1行=1フレームのCSV（time_s, datetime, lat, lon,
      quat_w/x/y/z, gyro_x/y/z ...）
"""
import struct
import sys
import csv
import json
import argparse
from pathlib import Path


# ---------------------------------------------------------------------------
# MP4 (ISO BMFF) box parsing
# ---------------------------------------------------------------------------
CONTAINER_BOXES = {b"moov", b"trak", b"mdia", b"minf", b"stbl", b"udta", b"edts"}


def iter_boxes(buf, start, end):
    """指定範囲の直下の box を (type, box_start, box_size, payload_start) で返す。"""
    off = start
    while off + 8 <= end:
        size = struct.unpack_from(">I", buf, off)[0]
        typ = buf[off + 4:off + 8]
        header = 8
        if size == 1:  # 64bit size
            size = struct.unpack_from(">Q", buf, off + 8)[0]
            header = 16
        elif size == 0:  # 末尾まで
            size = end - off
        yield typ, off, size, off + header
        if size <= 0:
            break
        off += size


def find_all(buf, typ, start=0, end=None):
    """再帰的に指定 type の box を全部見つける。"""
    if end is None:
        end = len(buf)
    found = []
    for t, o, s, p in iter_boxes(buf, start, end):
        if t == typ:
            found.append((o, s, p))
        if t in CONTAINER_BOXES:
            found += find_all(buf, typ, p, o + s)
    return found


def get_child(buf, parent_payload, parent_end, typ):
    for t, o, s, p in iter_boxes(buf, parent_payload, parent_end):
        if t == typ:
            return o, s, p
    return None


# ---------------------------------------------------------------------------
# トラック探索: handler / codec からメタデータトラックを見つける
# ---------------------------------------------------------------------------
def find_metadata_traks(buf):
    """(handler_name, codec, stbl_payload, stbl_end, timescale) のリストを返す。"""
    result = []
    for o, s, p in find_all(buf, b"trak"):
        trak_end = o + s
        mdia = get_child(buf, p, trak_end, b"mdia")
        if not mdia:
            continue
        mo, ms, mp = mdia
        mdia_end = mo + ms
        # hdlr -> handler name
        hdlr = get_child(buf, mp, mdia_end, b"hdlr")
        handler_name = ""
        if hdlr:
            ho, hs, hp = hdlr
            hdata = buf[hp:ho + hs]
            handler_name = hdata[24:].split(b"\x00")[0].decode("latin1", "replace")
        # mdhd -> timescale
        mdhd = get_child(buf, mp, mdia_end, b"mdhd")
        timescale = None
        if mdhd:
            do, ds, dp = mdhd
            timescale = struct.unpack_from(">I", buf, dp + 12)[0]
        # minf/stbl
        minf = get_child(buf, mp, mdia_end, b"minf")
        if not minf:
            continue
        io, is_, ip = minf
        stbl = get_child(buf, ip, io + is_, b"stbl")
        if not stbl:
            continue
        bo, bs, bp = stbl
        # codec (stsd 内の先頭エントリ type)
        stsd = get_child(buf, bp, bo + bs, b"stsd")
        codec = ""
        if stsd:
            so, ss, sp = stsd
            # stsd payload: ver/flags(4)+count(4)+entry_size(4)+format(4)
            codec = buf[sp + 12:sp + 16].decode("latin1", "replace")
        result.append({
            "handler": handler_name, "codec": codec,
            "stbl_payload": bp, "stbl_end": bo + bs, "timescale": timescale,
        })
    return result


def read_sample_table(buf, stbl_payload, stbl_end):
    """stsz/stco/stsc/stts から各サンプルの (file_offset, size, duration) を復元。"""
    def child(t):
        return get_child(buf, stbl_payload, stbl_end, t)

    # stsz
    o, s, p = child(b"stsz")
    sample_size = struct.unpack_from(">I", buf, p + 4)[0]
    count = struct.unpack_from(">I", buf, p + 8)[0]
    if sample_size == 0:
        sizes = [struct.unpack_from(">I", buf, p + 12 + 4 * i)[0] for i in range(count)]
    else:
        sizes = [sample_size] * count

    # chunk offsets (stco / co64)
    co = child(b"stco")
    if co:
        o, s, p = co
        n = struct.unpack_from(">I", buf, p + 4)[0]
        chunk_offsets = [struct.unpack_from(">I", buf, p + 8 + 4 * i)[0] for i in range(n)]
    else:
        o, s, p = child(b"co64")
        n = struct.unpack_from(">I", buf, p + 4)[0]
        chunk_offsets = [struct.unpack_from(">Q", buf, p + 8 + 8 * i)[0] for i in range(n)]

    # stsc: sample-to-chunk
    o, s, p = child(b"stsc")
    n = struct.unpack_from(">I", buf, p + 4)[0]
    stsc = [struct.unpack_from(">III", buf, p + 8 + 12 * i) for i in range(n)]

    # 各チャンクのサンプル数を展開
    samples_per_chunk = []
    for i, (first, spc, _desc) in enumerate(stsc):
        last = stsc[i + 1][0] - 1 if i + 1 < len(stsc) else len(chunk_offsets)
        for _c in range(first, last + 1):
            samples_per_chunk.append(spc)
    while len(samples_per_chunk) < len(chunk_offsets):
        samples_per_chunk.append(samples_per_chunk[-1] if samples_per_chunk else 1)

    # サンプルの file offset を算出
    offsets = []
    si = 0
    for ci, coff in enumerate(chunk_offsets):
        spc = samples_per_chunk[ci] if ci < len(samples_per_chunk) else 1
        pos = coff
        for _ in range(spc):
            if si >= count:
                break
            offsets.append(pos)
            pos += sizes[si]
            si += 1

    # stts: durations
    o, s, p = child(b"stts")
    n = struct.unpack_from(">I", buf, p + 4)[0]
    durations = []
    for i in range(n):
        cnt, dur = struct.unpack_from(">II", buf, p + 8 + 8 * i)
        durations += [dur] * cnt
    while len(durations) < count:
        durations.append(durations[-1] if durations else 0)

    return list(zip(offsets, sizes[:len(offsets)], durations[:len(offsets)]))


# ---------------------------------------------------------------------------
# 汎用 protobuf デコーダ（.proto スキーマ不要）
# ---------------------------------------------------------------------------
def read_varint(b, i):
    shift = 0
    res = 0
    while True:
        x = b[i]
        i += 1
        res |= (x & 0x7F) << shift
        if not x & 0x80:
            break
        shift += 7
    return res, i


def pb_parse(b):
    """protobuf メッセージを {field: [values...]} に。values は wire-typeごとの生バイト/int。"""
    i = 0
    n = len(b)
    out = {}
    while i < n:
        try:
            key, i = read_varint(b, i)
        except IndexError:
            break
        field = key >> 3
        wt = key & 7
        if wt == 0:
            v, i = read_varint(b, i)
        elif wt == 1:
            v = b[i:i + 8]
            i += 8
        elif wt == 5:
            v = b[i:i + 4]
            i += 4
        elif wt == 2:
            ln, i = read_varint(b, i)
            v = b[i:i + ln]
            i += ln
        else:
            break
        out.setdefault(field, []).append(v)
    return out


def f32(b):
    return struct.unpack("<f", b)[0]


def f64(b):
    return struct.unpack("<d", b)[0]


# ---------------------------------------------------------------------------
# dvtm_ac206 (Osmo Action 6) フレームテレメトリの意味づけ
# ---------------------------------------------------------------------------
def decode_frame(sample_bytes):
    """1サンプル(protobuf)からテレメトリ dict を抽出。未知フィールドは黙って無視。"""
    rec = {}
    top = pb_parse(sample_bytes)
    if 3 not in top:
        return rec  # メタデータ(先頭)サンプルなど
    m3 = pb_parse(top[3][0])

    # --- ブロックA (#2): 姿勢/IMU ---
    if 2 in m3:
        blkA = pb_parse(m3[2][0])
        # #9 = 姿勢クォータニオン (w, x, y, z)
        if 9 in blkA:
            q = pb_parse(blkA[9][0])
            for name, fid in (("quat_w", 1), ("quat_x", 2), ("quat_y", 3), ("quat_z", 4)):
                if fid in q:
                    rec[name] = f32(q[fid][0])
        # #10 = 3軸ベクトル（角速度 or 加速度と推定）
        if 10 in blkA:
            v = pb_parse(blkA[10][0])
            for name, fid in (("vec_x", 2), ("vec_y", 3), ("vec_z", 4)):
                if fid in v:
                    rec[name] = f32(v[fid][0])

    # --- ブロックB (#4): GPS / 日時 / 高度など ---
    if 4 in m3:
        blkB = pb_parse(m3[4][0])
        if 2 in blkB:
            b2 = pb_parse(blkB[2][0])
            if 1 in b2:
                gps = pb_parse(b2[1][0])
                if 2 in gps:
                    rec["lat"] = f64(gps[2][0])
                if 3 in gps:
                    rec["lon"] = f64(gps[3][0])
            if 6 in b2:  # 日時文字列
                dt = pb_parse(b2[6][0])
                if 1 in dt:
                    rec["datetime_utc"] = dt[1][0].decode("latin1", "replace")
        if 3 in blkB:
            b3 = pb_parse(blkB[3][0])
            # 2つの float（高度/その他）。意味は暫定で aux1/aux2 として出力
            if 1 in b3:
                rec["aux1"] = f32(b3[1][0])
            if 2 in b3:
                rec["aux2"] = f32(b3[2][0])
    return rec


# ---------------------------------------------------------------------------
# メイン
# ---------------------------------------------------------------------------
def extract(path):
    buf = Path(path).read_bytes()
    traks = find_metadata_traks(buf)

    # djmd (CAM meta) トラックを選ぶ
    meta = next((t for t in traks if t["codec"] == "djmd"), None)
    if meta is None:
        raise SystemExit(
            "djmd (DJI metadata) トラックが見つかりません。検出トラック: "
            + ", ".join(f"{t['handler']}/{t['codec']}" for t in traks)
        )

    table = read_sample_table(buf, meta["stbl_payload"], meta["stbl_end"])
    timescale = meta["timescale"] or 30000

    rows = []
    t_ticks = 0
    for off, size, dur in table:
        rec = decode_frame(buf[off:off + size])
        rec["time_s"] = round(t_ticks / timescale, 4)
        t_ticks += dur
        if len(rec) > 1:  # time_s 以外に中身がある行のみ
            rows.append(rec)
    return rows, meta


def main():
    ap = argparse.ArgumentParser(description="DJI 動画テレメトリ抽出")
    ap.add_argument("video", help="DJI の .MP4 ファイル")
    ap.add_argument("-o", "--out", help="出力CSVパス（省略時は <video>.telemetry.csv）")
    ap.add_argument("--json", help="JSON でも出力する場合のパス")
    args = ap.parse_args()

    rows, meta = extract(args.video)
    print(f"検出: handler='{meta['handler']}' codec='{meta['codec']}' "
          f"timescale={meta['timescale']}  サンプル数={len(rows)}")

    out = args.out or (str(Path(args.video).with_suffix("")) + ".telemetry.csv")
    # 列順を固定
    cols = ["time_s", "datetime_utc", "lat", "lon",
            "quat_w", "quat_x", "quat_y", "quat_z",
            "vec_x", "vec_y", "vec_z", "aux1", "aux2"]
    cols = [c for c in cols if any(c in r for r in rows)]
    with open(out, "w", newline="", encoding="utf-8") as fp:
        w = csv.DictWriter(fp, fieldnames=cols)
        w.writeheader()
        for r in rows:
            w.writerow({c: r.get(c, "") for c in cols})
    print(f"CSV 書き出し: {out}")

    if args.json:
        Path(args.json).write_text(json.dumps(rows, ensure_ascii=False, indent=2),
                                   encoding="utf-8")
        print(f"JSON 書き出し: {args.json}")

    if rows:
        print("\n--- 先頭3行 ---")
        for r in rows[:3]:
            print({k: r.get(k) for k in cols})


if __name__ == "__main__":
    main()
