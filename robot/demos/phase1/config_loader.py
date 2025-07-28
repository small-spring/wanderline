import yaml
from pathlib import Path


class ConfigLoader:
    def __init__(self, config_path: str):
        self.config_path = Path(config_path)

    def load_config(self) -> dict:
        """Load configuration from YAML file."""

        if not self.config_path.exists():
            raise FileNotFoundError(f"❌ Configuration file not found: {self.config_path}")        

        with open(self.config_path, 'r') as file:
            try:
                config = yaml.safe_load(file)
            except yaml.YAMLError as e:
                raise yaml.YAMLError(f"❌ YAML parse error:{e}")
            
        if config is None:
            raise ValueError(f"❌ YAML file is empty.")
        
        if 'phase1' not in config:
            raise KeyError(f"❌ Missing 'phase1' section in config file: {self.config_path}")
        
        return config['phase1']
