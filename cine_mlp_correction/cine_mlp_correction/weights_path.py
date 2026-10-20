import os


def _installed_share_directory(package_name):
    from ament_index_python.packages import get_package_share_directory

    return get_package_share_directory(package_name)


def resolve_weights_dir(configured_dir, share_directory_of=_installed_share_directory):
    """Directory holding ``mlp_correction.onnx`` and ``scaler.json``.

    An empty setting means the copy installed with this package, which
    ``tools/download_assets.py`` fills before ``colcon build``.
    """
    configured_dir = str(configured_dir).strip()
    if configured_dir:
        return configured_dir

    return os.path.join(share_directory_of('cine_mlp_correction'), 'weights')
