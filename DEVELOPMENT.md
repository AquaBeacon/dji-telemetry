# 開発ガイド

DJI動画メタデータ抽出。

## セットアップ

Python標準ライブラリのみ。python extract_dji_telemetry.py Source/input.MP4 -o output.csv --json output.json で抽出します。

Pythonの依存を追加する場合は、このリポジトリ専用の仮想環境を推奨します。

```powershell
python -m venv .venv
.venv\Scripts\Activate.ps1
```

## 入口

- [extract_dji_telemetry.py](extract_dji_telemetry.py)

## 検証

```sh
python extract_dji_telemetry.py --help
```

## データと関連リポジトリ

このディレクトリがGitリポジトリのルートです。親の AquaBeacon-OSS は複数リポジトリの置き場です。横断開発では各リポジトリを同じ親ディレクトリに配置し、[全体の入口](https://github.com/AquaBeacon/workspace-tools)から関連設計へ進みます。単体checkoutでも本書のローカル入口を使用できます。

生データ・生成物・SDK・仮想環境は手元に保持し、公開対象を.gitignoreで区別します。追跡済みのファイルは.gitignoreだけでは除外されないので、`git status --short`と`git ls-files`を確認します。

ライセンスは[Apache 2.0](LICENSE)、第三者部分は[THIRD_PARTY.md](THIRD_PARTY.md)。

## 公開リポジトリ

GitHub: https://github.com/AquaBeacon/dji-telemetry

```sh
git clone https://github.com/AquaBeacon/dji-telemetry.git
```

公開ブランチはmainです。新設リポジトリは公開対象だけを初期履歴にし、過去の開発履歴と実験ログはローカルに保管します。
