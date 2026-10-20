#!/usr/bin/env python3
"""Download the data files kept out of git from the GitHub Release matching VERSION.

Each asset lands at the path its reader expects inside this repository (or under
`--dest`): the recorded trajectories under `data/trajectories/`. A file already present
with the recorded checksum is skipped; one present with a different checksum is refused
rather than overwritten. The MLP correction weights are not distributed.

Usage:
    python3 tools/download_assets.py                                       # every asset
    python3 tools/download_assets.py --only baseline_sim_20251227_031503.csv  # one asset (repeatable)
"""

import argparse
import hashlib
import sys
import urllib.request
from dataclasses import dataclass
from pathlib import Path

GITHUB_REPO = 'Kino-Studios/CINE'
REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
VERSION_FILE = REPOSITORY_ROOT / 'VERSION'


@dataclass(frozen=True)
class Asset:
    name: str
    sha256: str | None
    destination: str


def _assets_in(destination, sha256_by_name):
    return tuple(Asset(name, sha256, destination) for name, sha256 in sha256_by_name.items())


# Release assets are flat, so every name must be unique across destinations.
ASSETS = (
    *_assets_in(
        'data/trajectories/baseline',
        {
            'baseline_sim_20251227_031503.csv': '8f6cb4ae2d92f8bda90fc4934293a550274a4f00f0f84db24bd99506a273eee6',
            'baseline_sim_20251227_031555.csv': '7d58fc0ce37840d4e65a0a7a27e2ec1246f71a65411874f7ed92a0fa6b989c6f',
            'baseline_sim_20251227_032035.csv': 'c936f421bf08866dcaf87fb35f2fe36e0136c7444fda23aa6cbc2112801221a8',
            'baseline_sim_20251227_032132.csv': '6524397a64641d219e92df32708e2ce747451ae0e0cc24eadd57ea8173a394d5',
            'baseline_sim_20251227_032203.csv': 'c89b47d511acb976ed1d8a4558938d753e2cb142dbef998006ceb15680b69a6f',
            'baseline_sim_20251227_032234.csv': '7eefee64f7436ddb17e1337280dd4d8a7da6399ecd1d93d89882f19b6b591d37',
            'baseline_sim_20251227_032300.csv': '27a29ad84ca425c03f4e936abdb68ee8c9b937e9e5fcc2d5a65493de6efc67ea',
            'baseline_sim_20251227_032332.csv': '3f48d02994aafcc711d9d118b33eeb7ad5f893c1308620f57e03b4137ce55ba5',
            'baseline_sim_20251227_032438.csv': 'e3647791af8348a9581e61c654c9fcdb5672a04840389e273ee61addef7d0487',
            'baseline_sim_20251227_032545.csv': '877c1313b68fe8b734f9dac4e752aadb901779616e16e442848f36a32c4d2b61',
            'baseline_sim_20251227_032617.csv': '476cf204c476afc65533253e39a5be97fa4f35228b626a1cecd38030f866edb7',
            'baseline_sim_20251227_032659.csv': '1000727fadc1054acacbdac3c85868b308aaa7d0207284a1173be027c35bc286',
            'baseline_sim_20251227_032739.csv': 'cb25aa1134e78c3c07591bb069ab581e3b7e2bc54cd8414cf7c12a4c90afcce8',
            'baseline_sim_20251227_032949.csv': '43979b778176c26c8151082925b20d44b5ff6ba127612a5f1663729bd1b5eb09',
            'baseline_sim_20251227_033016.csv': '43756cc717eae86ee43796a60e424ef88e1c99da9f111ac1fa1e261336feba4f',
        },
    ),
    *_assets_in(
        'data/trajectories/cinematic',
        {
            'cinematic_sim_20251231_165604.csv': 'd7ad8e882f11c2cbfc797ca5d0a49429829da2350d4732c5fe905dfa5a53916e',
            'cinematic_sim_20251231_165844.csv': '6c28025e9f1602a0c38032956c3cd138a7046b74129449435f8f1e63904428b5',
            'cinematic_sim_20251231_165946.csv': 'f6569e7a389ea5038b59871303f2bd924bd6509679af2f647a4108f0945790a5',
            'cinematic_sim_20251231_170037.csv': '9e57d7e0958a76f69cd433f9340b8a63f9932e254ec8a92334ffb60b41c360e0',
            'cinematic_sim_20251231_170125.csv': '9b5282ae96901fb45b0b8628c775107f4314aa094fb87d062efbb3a37f40969a',
            'cinematic_sim_20251231_170712.csv': 'b90a50c5962f1711b945b6a4cf0e2c1b935f77dad08c2e875f500ce7eb54d2ec',
            'cinematic_sim_20251231_170804.csv': 'a1534452f1963652e16cc391854998449bc3a679361fe5a0c77f8665c135e70c',
            'cinematic_sim_20251231_170857.csv': 'e5c3ecdb9113ad0af66762ee3eac06439525732b3b08fad96b5bd1f37e3d4225',
            'cinematic_sim_20251231_170946.csv': '17fba86d3b838b51e23cfd2860742ead92a775328f9534ae8fa94407ffd1c3cd',
            'cinematic_sim_20251231_171038.csv': '926913de152c9959f464cbf2046c9fe101867be70ebc68ae182b54a9f775fb21',
            'cinematic_sim_20251231_171819.csv': '8ac879f4c71b58c7611f8f149a6eb42320daa41e47800a3adb4487586dfd4c8f',
            'cinematic_sim_20251231_171916.csv': 'a0e3c6083ba9b3af50d5a4dfcbc2a2272b926ef58bbff712c3e07c76b82c18a6',
            'cinematic_sim_20251231_172031.csv': '870edc3b77beb901ab25efcbfaf2351b34c4d6ca25cd73bbb910b9cc2591f12c',
            'cinematic_sim_20251231_172118.csv': 'af3807eea5bc62967925ffa2a675e1ae59ff0c3ec790e0b3b05f90b8aa6f7964',
            'cinematic_sim_20251231_172324.csv': 'e6c17e90c5c38e1cffaf97d36118b12ff0030eefafcd8315aa68fbf4ce33c73b',
        },
    ),
    *_assets_in(
        'data/trajectories',
        {
            'baseline_mean_std_trajectory.csv': '10b7830eb67f0deaece508165853a330dab913624e34ca970c68e76ff48a57e9',
            'cinematic_mean_std_trajectory.csv': '1ba87fb6b5402a582f6e3c2d2292b016d1eb315c9491e9c6177922be265bf57e',
        },
    ),
    *_assets_in(
        'data/trajectories/2026-01',
        {
            'baseline_sim_20260110_110141.csv': '4d0b2d14999708cdbf80607ab4e6cfa006474717134a866c71cea63784367efb',
            'baseline_sim_20260110_110238.csv': 'bcd4b8ae992b1edaeaa2af30d95ffbe86561ddf5fbec5513bc8b3566547bb59b',
            'baseline_sim_20260110_110310.csv': 'e81c5b198faf74492f51aec3e430444a1b8954c3a723e30df45179fe22008490',
            'baseline_sim_20260110_110404.csv': '63fbad2309650fc6be9d3d262c12a6ee3db77ce5e1328ad66d1359c8e8de7585',
            'baseline_sim_20260110_110444.csv': '4ca675dbb4e562795d5c572d0f2387078f52a4e196c5d8f69ef2b9a3efed6a5b',
            'distance_baseline_20260106_182937.csv': '5d1b58b10cd6eac6694a28bc7e85581ebd310f189841252919c9d486d5917f71',
            'distance_baseline_20260106_194043.csv': '34b1a58d2a5d6d59734a41cf99ae1695f172ea9e5f8d815d7d2e19f9d7e7b1fb',
            'distance_baseline_20260106_194136.csv': 'fd987a24c7da344597ad392a10a45b4f726e8160ffda2addec9ccfee33748a9c',
            'distance_baseline_20260106_194217.csv': '3533087257fba327b1603850146bdb61c3afde9c5255d314c11e0cbaa4fa18bd',
            'distance_baseline_20260106_194306.csv': '23a91082faab9dd0778f33eb98858c7c0ab34ec0c6abf6d971589775b70d49f2',
            'distance_baseline_20260106_194401.csv': 'baeb68b736518fb96e54cc6d587b6e7b026d8f5849ea9d22bfb5e979754185a5',
            'distance_baseline_20260106_194448.csv': '7e77c83665821bfda1a00b042d711e97a965bfb0b974d3b0bd436f5c9150724a',
            'distance_baseline_20260106_194951.csv': '91942dafb30c28ef3f6c31e52c25dfde8ec8ca7a04918ea1f1f025769a60804b',
            'distance_baseline_20260106_195121.csv': '394a6ba7383a2478059cfb3bd8eabbfbb748227dbea9581b62655c2b77711ab1',
            'distance_baseline_20260106_195406.csv': 'a99aeba94fdb2b33f169c8069423acd2249acba0984981c7338f73a47694950f',
            'distance_baseline_20260106_195446.csv': 'dfa29c4413e2f5cd8742403ee87107421e8108c7e0cbf902549abf5ac1ef3a4a',
            'distance_baseline_20260106_195544.csv': '54d9ceec694f83075433d2b42eb1ede77bcc0ebff01fe15611915471ee5ba73f',
            'distance_baseline_20260106_195628.csv': 'f36dfb3005361ba2995816506744a6859dfc78d006f69a992c7769a48aa33f9a',
            'distance_baseline_20260106_200704.csv': '051d85bacb1a02fc493c577a2373a739b39af4d708413b5a76bf3a44b1998d9a',
            'distance_baseline_20260106_200756.csv': '81d4b5e834590476d5802abf4bd834dcc4c5e575cecd9309aaa010e97732cde5',
            'distance_baseline_20260110_110310.csv': 'c3c4dfbaa2a1ed92b461b9129e30bb7143e96c3618fff5b4b067efbce818ef69',
            'distance_baseline_20260110_110404.csv': 'a345f22a759a66b994d88c8424370fd3ef579292847c4eb5e7458c69f6cccf92',
            'distance_constrained_20260106_223606.csv': 'bf817702c2dfd903ab819c0225fbeea62227205bb6e99609e9d8ec8925aa088c',
            'distance_constrained_20260106_223723.csv': '476e9cf7a50adba77dfd11ed1f498123a66ce8531492b559860514b66826e976',
            'distance_constrained_20260106_223829.csv': '6c7b8534306abccbb5919679ee57823938a401f180d963718b2ead9810665cc2',
            'distance_constrained_20260106_223926.csv': '23fa9a1aee866ea326be4a67601c985187612ac1017b4141150bb26068f6cf2c',
            'distance_constrained_20260106_224030.csv': 'ac918d411a29a8f7ebaa56bd8b472f65fd312e270eded2ae6364984ec3232d06',
            'distance_constrained_20260106_224240.csv': '02895988876e71aa4fd56b5f29237438fa0b4428b945543aed3d507ef43d8ee6',
            'distance_constrained_20260106_224348.csv': 'aff3dd27e527745ebbc892b124981152331ecbbbe5057370fe95ae6e33d5e35a',
            'distance_constrained_20260106_224640.csv': '3c1d731d796ddf899cc8d8c426aaef377a593bb59e6d5ae51c59ee1896be5d6a',
            'distance_constrained_20260106_224801.csv': '66754f6151747b1a40acd4b7057499e17e3cf35ceb595751e84f6e4d814e5908',
            'distance_constrained_20260106_224904.csv': 'cb82e58707ecf51afbf6fdbc6b2cfccac0389b431984f0af4d3414208744a22b',
            'distance_constrained_20260106_225010.csv': 'd79d43fc50487da522abd5af8bf8814a87cb18103af5411ef4ed03e5c443601c',
            'distance_constrained_20260106_225104.csv': 'ec2a1081352bd2b0462cde790f47743366e92ffd29b15548fda42d63e1cf7f98',
            'distance_constrained_20260106_225203.csv': '083c2b0123eeb3a263a7bfd3f0f6639d7dcd71bcc9a4a02e114989d15b2ec33a',
            'distance_constrained_20260106_225256.csv': 'dd6bf8fc7f6c992a1795f633bfbfa4916ff4660828b816e690252e3866681095',
            'distance_constrained_20260106_225351.csv': '1c97a4545f17f1669a92f2e3c54c2c0c1513c3cdd725e6405545a41e5079597e',
        },
    ),
)


class ChecksumMismatchError(Exception):
    pass


def release_tag(version_file=VERSION_FILE):
    return 'v' + Path(version_file).read_text().strip()


def file_sha256(path):
    digest = hashlib.sha256()
    with open(path, 'rb') as file_handle:
        for chunk in iter(lambda: file_handle.read(1 << 20), b''):
            digest.update(chunk)

    return digest.hexdigest()


def _download_with_urllib(url, target_path):
    urllib.request.urlretrieve(url, target_path)


def fetch_asset(asset, destination_root, tag, download=_download_with_urllib):
    """Place `asset` under `destination_root`, downloading it only when absent.

    Raises ChecksumMismatchError when a present file or a fresh download does not
    match the recorded checksum; a present file is never overwritten.
    """
    target_path = Path(destination_root) / asset.destination / asset.name
    if target_path.exists():
        if asset.sha256 is not None and file_sha256(target_path) != asset.sha256:
            raise ChecksumMismatchError(
                f'{target_path} exists but its sha256 is not {asset.sha256}; '
                'not overwriting it. Delete the file to download it again.'
            )

        print(f'present: {target_path}')

        return

    target_path.parent.mkdir(parents=True, exist_ok=True)
    partial_path = target_path.with_name(target_path.name + '.part')
    url = f'https://github.com/{GITHUB_REPO}/releases/download/{tag}/{asset.name}'
    print(f'downloading {url}')
    try:
        download(url, partial_path)
        if asset.sha256 is not None and file_sha256(partial_path) != asset.sha256:
            raise ChecksumMismatchError(
                f'downloaded {asset.name} does not match sha256 {asset.sha256}; discarded'
            )
    except BaseException:
        partial_path.unlink(missing_ok=True)

        raise

    partial_path.rename(target_path)
    if asset.sha256 is None:
        print(f'warning: no checksum recorded for {asset.name}; not verified')
    print(f'done: {target_path}')


def main(argv):
    asset_names = [asset.name for asset in ASSETS]
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument(
        '--only',
        action='append',
        choices=asset_names,
        metavar='NAME',
        help=f'download only this asset (repeatable); one of: {", ".join(asset_names)}',
    )
    parser.add_argument(
        '--dest',
        type=Path,
        default=REPOSITORY_ROOT,
        help='root the asset destinations are relative to (default: this repository)',
    )
    arguments = parser.parse_args(argv)

    tag = release_tag()
    selected_assets = [
        asset for asset in ASSETS if not arguments.only or asset.name in arguments.only
    ]
    for asset in selected_assets:
        try:
            fetch_asset(asset, arguments.dest, tag)
        except (ChecksumMismatchError, OSError) as error:
            print(f'error: {error}', file=sys.stderr)

            return 1

    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
