import os


def download_gdrive(field, output_path):
    cmd = f"""wget --load-cookies ~/cookies.txt "https://docs.google.com/uc?export=download&confirm=$(wget --quiet --save-cookies ~/cookies.txt --keep-session-cookies --no-check-certificate 'https://docs.google.com/uc?export=download&id={field}' -O- | sed -rn 's/.*confirm=([0-9A-Za-z_]+).*/\\1\\n/p')&id={field}" -O {output_path} && rm -rf ~/cookies.txt"""
    os.system(cmd)

calib_ids = [
    '1nibGMReyFx1cNZCcWqeGdQaRZ-M--C1A',
    '1Cv0Rwd4e-HoZiA9hg8vMr-KuHYkuqLUo',
    '1XcfRpWv3bvQuLelYACGNgjBT4RMgwxkg',
]
act_ids = [
    '1505GHiNLzIIsxVlMe3R2VJCdhsjX1FJ1',
    '12UWYxzTxGc-A9NO1BH6Bm1oFuPTDQeQN',
    '1zoBRscrUdIVBEERjoGqj8TXQW46gSRjr',
]

if __name__ == '__main__':
    os.makedirs('./video/calib/', exist_ok=True)
    os.makedirs('./video/act/', exist_ok=True)

    for i, ci in enumerate(calib_ids):
        download_gdrive(ci, f'./video/calib/0{i+1}.avi')

    for i, ai in enumerate(act_ids):
        download_gdrive(ai, f'./video/act/0{i+1}.avi')