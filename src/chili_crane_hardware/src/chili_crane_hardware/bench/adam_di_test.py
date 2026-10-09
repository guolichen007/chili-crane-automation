"""One read of all raw DI; does not apply a guessed contact polarity."""
import argparse
import json
from chili_crane_hardware.adam.adam6052_driver import Adam6052Driver
from chili_crane_hardware.adam.adam6251_driver import Adam6251Driver
from .common import load_config, tcp_from_config


def main():
    parser = argparse.ArgumentParser(description="Read-only ADAM DI snapshot")
    parser.add_argument("--config", required=True)
    parser.add_argument("--model", choices=("adam6052", "adam6251"), required=True)
    args = parser.parse_args()
    driver_type = Adam6052Driver if args.model == "adam6052" else Adam6251Driver
    driver = driver_type(tcp_from_config(load_config(args.config)))
    print(json.dumps({"device": args.model, "raw_di": list(driver.read_di()),
                      "physical_output": False}))


if __name__ == "__main__":
    main()
