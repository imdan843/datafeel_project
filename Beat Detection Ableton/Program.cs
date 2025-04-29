using System;
using System.Collections.Generic;
using System.Net;
using System.Net.Sockets;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using Datafeel;
using Datafeel.NET.Serial;

enum MusicState
{
    None,
    Start1,
    Instrumental1,
    Verse1,
    Pause,
    Verse2,
    Bridge1,
    Bridge2,
    Bridge3,
    Bridge4,
    Pause1,  // Renamed from SpecialPause to Pause1
    Think,
    CrazyDrums,
    Strobing,
    End
}

class OSCBeatController
{
    private static DotManager? _manager;
    private static List<DotPropsWritable>? _dots;

    // Beat detection and debouncing
    private static bool _beatDetected = false;
    private static readonly object _beatLock = new object();
    private static DateTime _lastBeatTime = DateTime.MinValue;
    private static DateTime _lastBeatDetectedTime = DateTime.MinValue;
    private static int _beatDebounceMs = 150;

    // Current music state
    private static MusicState _currentMusicState = MusicState.None;
    private static readonly object _stateLock = new object();

    // Whether we are strobing
    private static bool _isStrobing = false;
    private static int _strobeIntervalMs = 150;

    // Verse1 "one-dot-at-a-time" index
    private static int _verse1DotIndex = 0;

    // Current LED color
    private static byte _currentRed = 0;
    private static byte _currentGreen = 0;
    private static byte _currentBlue = 0;

    // Current vibration intensity (0..1)
    private static float _currentVibrationIntensity = 0f;

    // For multi-color strobing in CrazyDrums
    private static readonly List<(byte r, byte g, byte b)> _crazyDrumColors = new()
    {
        (255, 0, 0),
        (0, 255, 0),
        (0, 0, 255),
        (255, 255, 0),
        (255, 0, 255),
        (0, 255, 255),
        (255, 128, 0),
        (128, 0, 255)
    };
    private static int _crazyDrumColorIndex = 0;

    static void Main(string[] args)
    {
        InitializeDatafeelHardware().Wait();
        Task.Run(() => StartOSCListener());
        Task.Run(() => HandleVibrations());

        Console.WriteLine("Beat-reactive system is running!");
        Console.ReadKey();
        Environment.Exit(0);
    }

    static async Task InitializeDatafeelHardware()
    {
        var manager = new DotManagerConfiguration()
            .AddDot<Dot_63x_xxx>(1)
            .AddDot<Dot_63x_xxx>(2)
            .AddDot<Dot_63x_xxx>(3)
            .AddDot<Dot_63x_xxx>(4)
            .CreateDotManager();

        _dots = new List<DotPropsWritable>()
    {
        new DotPropsWritable() {
            Address = 1,
            LedMode = LedModes.Off,
            GlobalLed = new(),
            VibrationMode = VibrationModes.Manual,
            VibrationIntensity = 0f,
            VibrationFrequency = 170,
            // Set coldest temperature
            ThermalMode = ThermalModes.Manual,
            ThermalIntensity = -1.0f,
            TargetSkinTemperature = 0f
        },
        new DotPropsWritable() {
            Address = 2,
            LedMode = LedModes.Off,
            GlobalLed = new(),
            VibrationMode = VibrationModes.Manual,
            VibrationIntensity = 0f,
            VibrationFrequency = 170,
            // Set coldest temperature
            ThermalMode = ThermalModes.Manual,
            ThermalIntensity = -1.0f,
            TargetSkinTemperature = 0f
        },
        new DotPropsWritable() {
            Address = 3,
            LedMode = LedModes.Off,
            GlobalLed = new(),
            VibrationMode = VibrationModes.Manual,
            VibrationIntensity = 0f,
            VibrationFrequency = 170,
            // Set coldest temperature
            ThermalMode = ThermalModes.Manual,
            ThermalIntensity = -1.0f,
            TargetSkinTemperature = 0f
        },
        new DotPropsWritable() {
            Address = 4,
            LedMode = LedModes.Off,
            GlobalLed = new(),
            VibrationMode = VibrationModes.Manual,
            VibrationIntensity = 0f,
            VibrationFrequency = 170,
            // Set coldest temperature
            ThermalMode = ThermalModes.Manual,
            ThermalIntensity = -1.0f,
            TargetSkinTemperature = 0f
        },
    };

        try
        {
            using var cts = new CancellationTokenSource(2000);
            var serialClient = new DatafeelModbusClientConfiguration()
                .UseWindowsSerialPortTransceiver()
                //.UseSerialPort("COM3")
                .CreateClient();
            var clients = new List<DatafeelModbusClient> { serialClient };

            bool result = await manager.Start(clients, cts.Token);

            if (result)
            {
                Console.WriteLine("Datafeel hardware successfully connected");
                _manager = manager;

                foreach (var dot in _dots)
                    await _manager.Write(dot);
            }
            else
            {
                Console.WriteLine("Failed to connect to Datafeel hardware");
            }
        }
        catch (Exception e)
        {
            Console.WriteLine($"Error initializing hardware: {e.Message}");
        }
    }


    static void StartOSCListener()
    {
        IPEndPoint localEndPoint = new IPEndPoint(IPAddress.Parse("127.0.0.1"), 5005);
        using UdpClient udpClient = new UdpClient(localEndPoint);

        try
        {
            while (true)
            {
                IPEndPoint remoteEndPoint = new IPEndPoint(IPAddress.Any, 0);
                byte[] data = udpClient.Receive(ref remoteEndPoint);
                if (data.Length > 0) ParseOSCMessage(data);
            }
        }
        catch (Exception ex)
        {
            Console.WriteLine($"OSC listener error: {ex.Message}");
        }
    }

    static void ParseOSCMessage(byte[] data)
    {
        try
        {
            string address = ReadOSCString(data, 0, out _);
            Console.WriteLine($"Received OSC address: '{address}'");

            // Debounce the beat
            if (address == "/prepend/beat" || address == "/prepend beat")
            {
                DateTime now = DateTime.Now;
                if ((now - _lastBeatDetectedTime).TotalMilliseconds > _beatDebounceMs)
                {
                    lock (_beatLock)
                    {
                        _beatDetected = true;
                        _lastBeatTime = now;
                    }
                    _lastBeatDetectedTime = now;
                    Console.WriteLine($"Beat detected at {now:HH:mm:ss.fff}");
                }
                return;
            }

            switch (address)
            {
                case "/start1":
                    SetMusicState(MusicState.Start1);
                    break;
                case "/instrumental1":
                    SetMusicState(MusicState.Instrumental1);
                    break;
                case "/verse1":
                    SetMusicState(MusicState.Verse1);
                    break;
                case "/pause":
                    SetMusicState(MusicState.Pause);
                    break;
                case "/verse2":
                    SetMusicState(MusicState.Verse2);
                    break;
                case "/bridge1":
                    SetMusicState(MusicState.Bridge1);
                    break;
                case "/bridge2":
                    SetMusicState(MusicState.Bridge2);
                    break;
                case "/bridge3":
                    SetMusicState(MusicState.Bridge3);
                    break;
                case "/bridge4":
                    SetMusicState(MusicState.Bridge4);
                    break;
                case "/pause1":
                    SetMusicState(MusicState.Pause1);
                    break;
                case "/think":
                    SetMusicState(MusicState.Think);
                    break;
                case "/crazydrums":
                    SetMusicState(MusicState.CrazyDrums);
                    break;
                case "/strobing":
                    SetMusicState(MusicState.Strobing);
                    break;
                case "/end":
                    SetMusicState(MusicState.End);
                    break;

                default:
                    Console.WriteLine($"Unknown OSC address: {address}");
                    break;
            }
        }
        catch (Exception ex)
        {
            Console.WriteLine($"Error parsing OSC: {ex.Message}");
        }
    }

    static string ReadOSCString(byte[] data, int startIndex, out int endIndex)
    {
        int i = startIndex;
        while (i < data.Length && data[i] != 0) i++;
        string result = Encoding.ASCII.GetString(data, startIndex, i - startIndex);

        i++;
        while (i % 4 != 0) i++;
        endIndex = i;
        return result;
    }

    static void SetMusicState(MusicState newState)
    {
        bool wasStrobing = false;

        lock (_stateLock)
        {
            wasStrobing = _isStrobing;
            _currentMusicState = newState;
            Console.WriteLine($"*** Transitioning to state: {_currentMusicState}");
            _isStrobing = false;
            _verse1DotIndex = 0;
            _strobeIntervalMs = 150;

            // Default color / intensities
            _currentRed = 0;
            _currentGreen = 0;
            _currentBlue = 0;
            _currentVibrationIntensity = 0f;

            switch (newState)
            {
                case MusicState.Start1:
                    _currentRed = 0; _currentGreen = 0; _currentBlue = 255;
                    _currentVibrationIntensity = 1.0f;
                    break;

                case MusicState.Instrumental1:
                    _currentRed = 255; _currentGreen = 255; _currentBlue = 255;
                    _currentVibrationIntensity = 0.2f;
                    break;

                case MusicState.Verse1:
                    _currentRed = 255; _currentGreen = 255; _currentBlue = 255;
                    _currentVibrationIntensity = 0.5f;
                    break;

                // CHANGED: /pause -> "breathing green red" => pick bright YELLOW
                // We can't do two-color breathing, so we combine them: R=255,G=255,B=0
                case MusicState.Pause:
                    _currentRed = 255;
                    _currentGreen = 255;
                    _currentBlue = 0;
                    _currentVibrationIntensity = 0f;
                    break;

                case MusicState.Verse2:
                    _currentRed = 255; _currentGreen = 0; _currentBlue = 0;
                    _currentVibrationIntensity = 0.7f;
                    break;

                case MusicState.Bridge1:
                    _currentRed = 255; _currentGreen = 255; _currentBlue = 255;
                    _currentVibrationIntensity = 0f;
                    break;

                // CHANGED: /bridge2 -> breathing green
                // set red=0, green=255, blue=0
                case MusicState.Bridge2:
                    _currentRed = 0;
                    _currentGreen = 255;
                    _currentBlue = 0;
                    _currentVibrationIntensity = 0f;
                    break;

                case MusicState.Bridge3:
                    _currentRed = 255; _currentGreen = 0; _currentBlue = 0;
                    _currentVibrationIntensity = 0.7f;
                    break;

                case MusicState.Bridge4:
                    _currentRed = 255; _currentGreen = 255; _currentBlue = 255;
                    _currentVibrationIntensity = 0.5f;
                    _isStrobing = true;
                    break;

                case MusicState.Pause1:
                    _currentRed = 255; _currentGreen = 0; _currentBlue = 0;
                    _currentVibrationIntensity = 0f;
                    break;

                case MusicState.Think:
                    _currentRed = 255; _currentGreen = 0; _currentBlue = 0;
                    _currentVibrationIntensity = 0.8f;
                    break;

                case MusicState.CrazyDrums:
                    _isStrobing = true;
                    _strobeIntervalMs = 80;
                    _currentVibrationIntensity = 1.0f;
                    break;

                case MusicState.Strobing:
                    _currentRed = 255; _currentGreen = 255; _currentBlue = 255;
                    _currentVibrationIntensity = 1.0f;
                    _isStrobing = true;
                    break;

                case MusicState.End:
                    _currentRed = 255; _currentGreen = 255; _currentBlue = 255;
                    _currentVibrationIntensity = 0f;
                    break;
            }
        }

        if (wasStrobing)
        {
            StopAllEffects().Wait();
            Thread.Sleep(50);
        }

        ApplyImmediateState();
    }

    static async Task StopAllEffects()
    {
        if (_dots == null || _manager == null) return;
        foreach (var dot in _dots)
        {
            dot.LedMode = LedModes.Off;
            dot.VibrationIntensity = 0.0f;
            await _manager.Write(dot);
        }
    }

    static void ApplyImmediateState()
    {
        if (_dots == null || _manager == null) return;

        if (_isStrobing)
        {
            StopAllEffects().Wait();
            return;
        }

        switch (_currentMusicState)
        {
            case MusicState.Start1:
            case MusicState.Pause:
            case MusicState.Pause1:
            case MusicState.End:
                foreach (var dot in _dots)
                {
                    dot.LedMode = LedModes.Breathe;
                    dot.GlobalLed.Red = _currentRed;
                    dot.GlobalLed.Green = _currentGreen;
                    dot.GlobalLed.Blue = _currentBlue;
                    dot.VibrationIntensity = _currentVibrationIntensity;
                    _manager.Write(dot);
                }
                break;

            // Bridge1 => breathing white, Bridge2 => breathing green
            case MusicState.Bridge1:
            case MusicState.Bridge2:
            case MusicState.Bridge3:
                foreach (var dot in _dots)
                {
                    dot.LedMode = LedModes.Breathe;
                    dot.GlobalLed.Red = _currentRed;
                    dot.GlobalLed.Green = _currentGreen;
                    dot.GlobalLed.Blue = _currentBlue;
                    dot.VibrationIntensity = _currentVibrationIntensity;
                    _manager.Write(dot);
                }
                break;

            case MusicState.Instrumental1:
            case MusicState.Verse2:
            case MusicState.Think:
                foreach (var dot in _dots)
                {
                    dot.LedMode = LedModes.GlobalManual;
                    dot.GlobalLed.Red = _currentRed;
                    dot.GlobalLed.Green = _currentGreen;
                    dot.GlobalLed.Blue = _currentBlue;
                    dot.VibrationIntensity = 0f;
                    _manager.Write(dot);
                }
                break;

            default:
                break;
        }
    }

    static async Task HandleVibrations()
    {
        while (true)
        {
            bool beatHappened = false;
            lock (_beatLock)
            {
                beatHappened = _beatDetected;
                _beatDetected = false;
            }

            MusicState currentState;
            bool isCurrentlyStrobing;

            lock (_stateLock)
            {
                currentState = _currentMusicState;
                isCurrentlyStrobing = _isStrobing;
            }

            if (isCurrentlyStrobing)
            {
                await DoStrobePass();
            }
            else if (beatHappened)
            {
                await HandleNormalBeat();
            }

            await Task.Delay(10);
        }
    }

    private static async Task DoStrobePass()
    {
        if (_dots == null || _manager == null) return;

        MusicState currentState;
        lock (_stateLock)
        {
            currentState = _currentMusicState;
            if (!_isStrobing) return;
        }

        if (currentState == MusicState.CrazyDrums)
        {
            for (int i = 0; i < _dots.Count; i++)
            {
                lock (_stateLock)
                {
                    if (!_isStrobing) return;
                }

                var dot = _dots[i];
                var color = _crazyDrumColors[_crazyDrumColorIndex];
                _crazyDrumColorIndex = (_crazyDrumColorIndex + 1) % _crazyDrumColors.Count;

                dot.LedMode = LedModes.GlobalManual;
                dot.GlobalLed.Red = color.r;
                dot.GlobalLed.Green = color.g;
                dot.GlobalLed.Blue = color.b;
                dot.VibrationIntensity = 1.0f;
                dot.VibrationFrequency = 170 + (dot.Address * 10);
                await _manager.Write(dot);

                await Task.Delay(_strobeIntervalMs);

                lock (_stateLock)
                {
                    if (!_isStrobing) return;
                }

                dot.LedMode = LedModes.Off;
                dot.VibrationIntensity = 0.0f;
                await _manager.Write(dot);
            }
        }
        else
        {
            float vibIntensity;
            lock (_stateLock)
            {
                vibIntensity = (_currentVibrationIntensity > 0) ? _currentVibrationIntensity : 1.0f;
            }

            for (int i = 0; i < _dots.Count; i++)
            {
                lock (_stateLock)
                {
                    if (!_isStrobing) return;
                }

                var dot = _dots[i];
                dot.LedMode = LedModes.GlobalManual;
                dot.GlobalLed.Red = _currentRed;
                dot.GlobalLed.Green = _currentGreen;
                dot.GlobalLed.Blue = _currentBlue;
                dot.VibrationIntensity = vibIntensity;
                dot.VibrationFrequency = 170 + (dot.Address * 10);
                await _manager.Write(dot);

                await Task.Delay(_strobeIntervalMs);

                lock (_stateLock)
                {
                    if (!_isStrobing) return;
                }

                dot.LedMode = LedModes.Off;
                dot.VibrationIntensity = 0.0f;
                await _manager.Write(dot);
            }
        }
    }

    private static async Task HandleNormalBeat()
    {
        if (_dots == null || _manager == null) return;

        MusicState currentState;
        lock (_stateLock)
        {
            currentState = _currentMusicState;
            if (_isStrobing) return;
        }

        // Skip beat handling for Bridge1 and Bridge2 states
        if (currentState == MusicState.Bridge1 || currentState == MusicState.Bridge2)
        {
            return; // no vibrations or blinking
        }

        switch (currentState)
        {
            case MusicState.Verse1:
                // sequential blinking
                for (int i = 0; i < _dots.Count; i++)
                {
                    var dot = _dots[i];
                    if (i == _verse1DotIndex)
                    {
                        dot.LedMode = LedModes.Breathe;
                        dot.GlobalLed.Red = _currentRed;
                        dot.GlobalLed.Green = _currentGreen;
                        dot.GlobalLed.Blue = _currentBlue;
                        dot.VibrationIntensity = _currentVibrationIntensity;
                        dot.VibrationFrequency = 170 + (dot.Address * 10);
                    }
                    else
                    {
                        dot.LedMode = LedModes.Off;
                        dot.VibrationIntensity = 0.0f;
                    }
                    await _manager.Write(dot);
                }
                _verse1DotIndex = (_verse1DotIndex + 1) % _dots.Count;

                await Task.Delay(150);

                // Turn off the one that was on
                int offIndex = (_verse1DotIndex - 1 + _dots.Count) % _dots.Count;
                var offDot = _dots[offIndex];
                offDot.LedMode = LedModes.Off;
                offDot.VibrationIntensity = 0.0f;
                await _manager.Write(offDot);
                break;

            case MusicState.Pause:
            case MusicState.Pause1:
            case MusicState.End:
                // No beat reaction
                break;

            default:
                // blink all
                foreach (var dot in _dots)
                {
                    dot.LedMode = LedModes.Breathe;
                    dot.GlobalLed.Red = _currentRed;
                    dot.GlobalLed.Green = _currentGreen;
                    dot.GlobalLed.Blue = _currentBlue;
                    dot.VibrationIntensity = _currentVibrationIntensity;
                    dot.VibrationFrequency = 170 + (dot.Address * 10);
                    await _manager.Write(dot);
                }

                await Task.Delay(150);

                foreach (var dot in _dots)
                {
                    dot.LedMode = LedModes.Off;
                    dot.VibrationIntensity = 0.0f;
                    await _manager.Write(dot);
                }
                break;
        }
    }
}
