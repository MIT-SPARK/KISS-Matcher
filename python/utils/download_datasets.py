import os
import shutil
import urllib.request
import zipfile

DATA_DIR = os.path.join(os.getcwd(), "data")
ZIP_DIR = os.path.join(os.getcwd(), "zip_files")

DROPBOX_FILES = [
    "https://www.dropbox.com/scl/fi/1lsz6bxwan0sytj87ea9h/Vel16.zip?rlkey=g4jpze2j6iu6hk9ahq0m6t7o3&st=vrfpk4nv&dl=1",
    "https://www.dropbox.com/scl/fi/weqfi572gbi5z654d56ai/Vel64.zip?rlkey=l9upmgfjx7nhkbgl9du7igrfu&st=4q9gyous&dl=1",
    "https://www.dropbox.com/scl/fi/lnsbaqmbgz0qi1r8ocesd/HeLiPR-KAIST05.zip?rlkey=50jyhl180qpmf1j5jn9ru37ru&st=q0kay7o3&dl=1",
    "https://www.dropbox.com/scl/fi/c6c1jxrld17ywj7x2ok1q/VBR-Collosseo.zip?rlkey=b1sk0xvnntqy8kyw1apob37ob&st=5imrvvwo&dl=1",
    "https://www.dropbox.com/scl/fi/73l59cj5ypfvjrkrc23qx/KITTI00-to-KITTI360.zip?rlkey=pqukxxgpxaq1pugo6dhzq8xa4&st=yns7uolj&dl=1",
    "https://www.dropbox.com/scl/fi/lb49afp7x5ja3bfptbo68/KITTI00-to-07.zip?rlkey=qkwq99vwwlnxnxj3nhdptrusr&st=nzx8ts9j&dl=1",
    "https://www.dropbox.com/scl/fi/n6sypvt4rdssn172mn2jv/bun_zipper.ply?rlkey=hk2danjxt29a7ahq374s8m7ak&st=o5udnqjv&dl=1",
]

DOWNLOADED_FILES = [
    os.path.join(ZIP_DIR, "Vel16.zip"),
    os.path.join(ZIP_DIR, "Vel64.zip"),
    os.path.join(ZIP_DIR, "HeLiPR-KAIST05.zip"),
    os.path.join(ZIP_DIR, "VBR-Collosseo.zip"),
    os.path.join(ZIP_DIR, "KITTI00-to-KITTI360.zip"),
    os.path.join(ZIP_DIR, "KITTI00-to-07.zip"),
    os.path.join(ZIP_DIR, "bun_zipper.ply"),
]

os.makedirs(DATA_DIR, exist_ok=True)
os.makedirs(ZIP_DIR, exist_ok=True)


def download_file(url, dest_file):
    print(f"Downloading {url} -> {dest_file}")
    urllib.request.urlretrieve(url, dest_file)
    print(f"Downloaded: {dest_file}")


def extract_zip(zip_path, extract_to):
    print(f"Extracting {zip_path} to {extract_to}")
    with zipfile.ZipFile(zip_path, "r") as zip_ref:
        zip_ref.extractall(extract_to)
    print(f"Extracted: {zip_path}")


def main():
    for url, dest_file in zip(DROPBOX_FILES, DOWNLOADED_FILES):
        if not os.path.exists(dest_file):
            download_file(url, dest_file)
        else:
            print(f"File already exists: {dest_file}, skipping download.")

        if dest_file.endswith(".zip"):
            extract_zip(dest_file, DATA_DIR)
        else:
            os.replace(dest_file,
                       os.path.join(DATA_DIR, os.path.basename(dest_file)))

    # Remove `ZIP_DIR` directory
    shutil.rmtree(ZIP_DIR, ignore_errors=True)
    print("✅ All datasets downloaded and extracted successfully!")


if __name__ == "__main__":
    main()
