try:
    import airsim
    print("AirSim imported successfully")
    print(f"AirSim file: {airsim.__file__}")
except ImportError:
    print("Failed to import airsim")
