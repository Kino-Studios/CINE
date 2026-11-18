from fetch_weights import verify_sha256

# sha256 of the three bytes b'abc' (FIPS 180-2 test vector).
ABC_SHA256 = 'ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad'


def test_file_matching_expected_hash_is_accepted(tmp_path):
    weights_file = tmp_path / 'weights.pt'
    weights_file.write_bytes(b'abc')

    assert verify_sha256(weights_file, ABC_SHA256) is True


def test_file_with_different_hash_is_rejected(tmp_path):
    weights_file = tmp_path / 'weights.pt'
    weights_file.write_bytes(b'abd')

    assert verify_sha256(weights_file, ABC_SHA256) is False
