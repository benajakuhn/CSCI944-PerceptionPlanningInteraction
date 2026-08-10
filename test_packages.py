from __future__ import annotations
import platform
import sys
import traceback

def run_test(name, test_function):
    try:
        test_function()
        print(f" [PASS] {name}")
        return True
    except Exception as exc:
        print(f" [FAIL] {name}: {exc}")
        traceback.print_exc()
        return False

def test_numpy():
    import numpy as np
    a = np.array([[1.0, 2.0], [3.0, 4.0]])
    assert (a @ a).shape == (2, 2)
    print("NumPy:", np.__version__)

def test_matplotlib():
    import matplotlib
    import matplotlib.pyplot as plt
    fig = plt.figure()
    plt.plot([0, 1, 2], [0, 1, 4])
    plt.close(fig)
    print("Matplotlib:", matplotlib.__version__)

def test_scipy():
    import scipy
    from scipy.linalg import det
    assert abs(det([[1.0, 2.0], [3.0, 4.0]]) + 2.0) < 1e-9
    print("SciPy:", scipy.__version__)

def test_scikit_learn():
    import sklearn
    from sklearn.linear_model import LinearRegression
    model = LinearRegression().fit([[0.0], [1.0], [2.0]], [0.0, 2.0, 4.0])
    assert abs(model.predict([[3.0]])[0] - 6.0) < 1e-6
    print("Scikit-learn:", sklearn.__version__)

def test_opencv():
    import cv2
    import numpy as np
    image = np.zeros((100, 100, 3), dtype=np.uint8)
    assert cv2.cvtColor(image, cv2.COLOR_BGR2GRAY).shape == (100, 100)
    assert cv2.SIFT_create() is not None
    print("OpenCV:", cv2.__version__)

def test_open3d():
    import numpy as np
    import open3d as o3d
    cloud = o3d.geometry.PointCloud()
    cloud.points = o3d.utility.Vector3dVector(np.array([[0, 0, 0], [1, 0, 0], [0, 1, 0]], dtype=float))
    assert len(cloud.points) == 3
    print("Open3D:", o3d.__version__)

def test_vosk():
    import vosk
    assert hasattr(vosk, "Model") and hasattr(vosk, "KaldiRecognizer")
    print("Vosk imported successfully")

def test_sounddevice():
    import sounddevice as sd
    print("Audio devices detected:", len(sd.query_devices()))

def test_pyttsx3():
    import pyttsx3
    engine = pyttsx3.init()
    print("TTS voices detected:", len(engine.getProperty("voices")))
    engine.stop()

def main():
    print("Python:", sys.version)
    print("Executable:", sys.executable)
    print("Architecture:", platform.architecture()[0])
    
    tests = [
        ("NumPy", test_numpy),
        ("Matplotlib", test_matplotlib),
        ("SciPy", test_scipy),
        ("Scikit-learn", test_scikit_learn),
        ("OpenCV contrib", test_opencv),
        ("Open3D", test_open3d),
        ("Vosk", test_vosk),
        ("sounddevice", test_sounddevice),
        ("pyttsx3", test_pyttsx3),
    ]
    
    passed = sum(run_test(name, fn) for name, fn in tests)
    print(f"\nPassed {passed} of {len(tests)} tests.")
    return 0 if passed == len(tests) else 1

if __name__ == "__main__":
    raise SystemExit(main())