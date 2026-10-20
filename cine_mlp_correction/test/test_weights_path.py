from cine_mlp_correction.weights_path import resolve_weights_dir


def _share_directory_of(package_name):
    return f'/opt/ws/install/{package_name}/share/{package_name}'


def test_configured_weights_dir_is_used_as_given():
    assert resolve_weights_dir('/data/mlp', _share_directory_of) == '/data/mlp'


def test_empty_weights_dir_falls_back_to_the_installed_package_share():
    assert (
        resolve_weights_dir('', _share_directory_of)
        == '/opt/ws/install/cine_mlp_correction/share/cine_mlp_correction/weights'
    )


def test_blank_weights_dir_falls_back_to_the_installed_package_share():
    assert (
        resolve_weights_dir('  ', _share_directory_of)
        == '/opt/ws/install/cine_mlp_correction/share/cine_mlp_correction/weights'
    )
