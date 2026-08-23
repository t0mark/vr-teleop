"""자체 서명 TLS 인증서 생성: WebXR secure context 요건용 (web/certs/에 출력).

사용법 (컨테이너 내부):
    python -m teleop.net.web_api.gen_certs [--ip 192.168.100.1 ...] [--days 825]

openssl 바이너리 대신 cryptography 패키지를 사용해 컨테이너 추가 의존성 없이 생성한다.
"""

import argparse
import datetime
import ipaddress
import logging

from cryptography import x509
from cryptography.hazmat.primitives import hashes, serialization
from cryptography.hazmat.primitives.asymmetric import rsa
from cryptography.x509.oid import NameOID

from teleop.utils import config
from teleop.utils.log import setup_logging

logger = logging.getLogger("gen_certs")


def generate_self_signed_cert(ips: list[str], days: int) -> tuple[bytes, bytes]:
    """자체 서명 인증서·개인키 PEM 바이트를 생성해 (cert, key) 튜플로 반환한다.

    ips: SAN에 넣을 서버 IP 목록 (Quest가 IP로 접속하므로 IP SAN 필수).
    days: 유효 기간(일).
    """
    key = rsa.generate_private_key(public_exponent=65537, key_size=2048)

    # Quest가 https://{IP}로 접속하므로 IP SAN이 없으면 예외 허용조차 불가한 브라우저가 있음.
    # localhost·127.0.0.1은 PC 로컬 테스트 편의를 위해 항상 포함
    san_entries = [x509.DNSName("localhost"), x509.IPAddress(ipaddress.ip_address("127.0.0.1"))]
    for ip in ips:
        san_entries.append(x509.IPAddress(ipaddress.ip_address(ip)))

    subject = issuer = x509.Name([x509.NameAttribute(NameOID.COMMON_NAME, "rby1-web-teleop")])
    now = datetime.datetime.now(datetime.timezone.utc)
    cert = (
        x509.CertificateBuilder()
        .subject_name(subject)
        .issuer_name(issuer)
        .public_key(key.public_key())
        .serial_number(x509.random_serial_number())
        .not_valid_before(now)
        .not_valid_after(now + datetime.timedelta(days=days))
        .add_extension(x509.SubjectAlternativeName(san_entries), critical=False)
        .sign(key, hashes.SHA256())
    )

    cert_pem = cert.public_bytes(serialization.Encoding.PEM)
    key_pem = key.private_bytes(
        serialization.Encoding.PEM,
        serialization.PrivateFormat.TraditionalOpenSSL,
        serialization.NoEncryption(),
    )
    return cert_pem, key_pem


def write_cert_files(ips: list[str], days: int):
    """인증서·키를 생성해 config에 정의된 경로(web/certs/)에 기록한다. 반환값 없음."""
    config.CERT_DIR.mkdir(parents=True, exist_ok=True)
    cert_pem, key_pem = generate_self_signed_cert(ips, days)
    config.CERT_FILE.write_bytes(cert_pem)
    config.KEY_FILE.write_bytes(key_pem)
    logger.info(f"인증서 생성 완료: {config.CERT_FILE} (SAN IP: {ips}, 유효 {days}일)")


def main():
    """CLI 인자를 파싱해 인증서를 생성한다."""
    parser = argparse.ArgumentParser(description="자체 서명 TLS 인증서 생성")
    parser.add_argument(
        "--ip", nargs="+", default=[config.DIRECT_LINK_SERVER_IP],
        help=f"SAN에 포함할 서버 IP (기본: {config.DIRECT_LINK_SERVER_IP})",
    )
    parser.add_argument("--days", type=int, default=825, help="유효 기간(일, 기본 825)")
    args = parser.parse_args()

    setup_logging()
    write_cert_files(args.ip, args.days)


if __name__ == "__main__":
    main()
