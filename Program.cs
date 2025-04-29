using System;
using System.Collections.Generic;
using System.IO;
using System.Threading;
using System.Threading.Tasks;
using System.Diagnostics;
using System.Net;
using System.Net.Sockets;
using System.Text;

// OpenCv for frame color
using OpenCvSharp;

// NAudio for reading MP4 audio
using NAudio.Wave;

// Datafeel
using Datafeel;
using Datafeel.NET.Serial;

class AudioVideoThermal
{
    private static DotManager? _manager;
    private static List<DotPropsWritable>? _dots;

    // --------------- AUDIO / FFT STUFF ---------------
    private static bool _stopAudio = false;          // signal to end audio loop
    private static int fftSize = 1024;               // buffer size for FFT
    private static float[] _audioBuffer = new float[fftSize];
    private static int _audioIndex = 0;
    private static int sampleRate = 44100;           // will detect from wave format
    private static int lowerFreqCutoff = 150;        // below 150Hz => triggers vibrations
    private static float beatThreshold = 0.3f;       // tune to your track’s volume
    private static bool _vibrationOn = false;        // current vibration state
    private static DateTime _lastVibrationOnTime = DateTime.MinValue;
    private static TimeSpan _vibrationHold = TimeSpan.FromMilliseconds(300);

    static async Task Main()
    {
        Console.WriteLine("=== MP4: Video => Thermal, Audio => Vibrations ===");

        // 1) Initialize Datafeel Dot hardware
        await InitializeDatafeelHardware();
        if (_manager == null || _dots == null)
        {
            Console.WriteLine("Failed to init Datafeel. Exiting...");
            return;
        }

        // 2) Ask user for MP4 path
        Console.Write("Path to MP4 file: ");
        string videoPath = Console.ReadLine()?.Trim('"');
        if (string.IsNullOrEmpty(videoPath) || !File.Exists(videoPath))
        {
            Console.WriteLine("File not found. Exiting...");
            return;
        }

        // 3) Launch Windows Media Player for on-screen playback
        Process? wmpProcess = null;
        try
        {
            string wmpFullPath = @"C:\Program Files\Windows Media Player\wmplayer.exe";
            wmpProcess = Process.Start(wmpFullPath, $"\"{videoPath}\"");
            Console.WriteLine("Launched Windows Media Player in separate window...");
        }
        catch (Exception ex)
        {
            Console.WriteLine($"Could not start WMP: {ex.Message}");
        }

        // 4) Start an audio analysis Task
        var audioTask = Task.Run(() => AnalyzeAudioFromMp4(videoPath));

        // 5) Meanwhile, read video frames with OpenCV
        using var capture = new VideoCapture(videoPath);
        if (!capture.IsOpened())
        {
            Console.WriteLine("OpenCV cannot open the video. Exiting...");
            _stopAudio = true;
            return;
        }

        double fps = capture.Fps;
        if (fps < 1) fps = 30; // fallback
        long totalFrames = (long)capture.Get(VideoCaptureProperties.FrameCount);

        Console.WriteLine("Reading video frames: controlling Dot thermal (R => heat, G+B => cool). Press ENTER to stop...");

        var stopwatch = Stopwatch.StartNew();
        var frame = new Mat();
        int currentFrame = 0;

        // Main loop: track color, set thermal
        while (true)
        {
            if (currentFrame >= totalFrames) break;
            if (Console.KeyAvailable && Console.ReadKey(true).Key == ConsoleKey.Enter) break;

            double elapsedSec = stopwatch.Elapsed.TotalSeconds;
            double targetFrame = elapsedSec * fps;
            if (currentFrame < targetFrame)
            {
                currentFrame = (int)Math.Floor(targetFrame);
                capture.Set(VideoCaptureProperties.PosFrames, currentFrame);
            }

            if (!capture.Read(frame) || frame.Empty()) break;
            currentFrame++;

            // (A) get average color
            (byte r, byte g, byte b) = GetAverageBGRColor(frame);

            // (B) compute thermal
            // formula: warmValue = R - (G+B).
            // scale up to ±3.0
            int warmValue = r - (g + b);
            float thermal = 0f;
            if (warmValue > 0)
            {
                float ratio = warmValue / 255f;
                thermal = Math.Min(3.0f, ratio * 3.0f);
            }
            else if (warmValue < 0)
            {
                // could be as low as -510
                float ratio = (float)warmValue / -510f;
                thermal = -Math.Min(3.0f, ratio * 3.0f);
            }

            Console.WriteLine($"Frame #{currentFrame}: R={r},G={g},B={b}, warmValue={warmValue}, Thermal={thermal:0.00}");

            // (C) apply color + thermal
            await UpdateDotsColorAndThermal(r, g, b, thermal);

            // small delay
            await Task.Delay(10);
        }

        Console.WriteLine("Stopping. Press any key to exit...");
        _stopAudio = true; // end audio thread
        stopwatch.Stop();

        await audioTask;   // wait for audio to finalize
        if (wmpProcess != null && !wmpProcess.HasExited)
        {
            // Optionally kill WMP:
            // wmpProcess.CloseMainWindow(); 
            // wmpProcess.Close();
        }

        await _manager.Stop();
        Console.ReadKey();
    }

    // -----------------------------------------------------------------------
    //  Initialize Datafeel with 4 Dots: default to cold & no vibration
    // -----------------------------------------------------------------------
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
            new DotPropsWritable {
                Address = 1,
                LedMode = LedModes.GlobalManual,
                GlobalLed = new RgbLed { Red=0, Green=0, Blue=0 },
                VibrationMode = VibrationModes.Manual,
                VibrationIntensity = 0f,
                VibrationFrequency = 170f,
                ThermalMode = ThermalModes.Manual,
                ThermalIntensity = -1.0f
            },
            new DotPropsWritable {
                Address = 2,
                LedMode = LedModes.GlobalManual,
                GlobalLed = new RgbLed { Red=0, Green=0, Blue=0 },
                VibrationMode = VibrationModes.Manual,
                VibrationIntensity = 0f,
                VibrationFrequency = 170f,
                ThermalMode = ThermalModes.Manual,
                ThermalIntensity = -1.0f
            },
            new DotPropsWritable {
                Address = 3,
                LedMode = LedModes.GlobalManual,
                GlobalLed = new RgbLed { Red=0, Green=0, Blue=0 },
                VibrationMode = VibrationModes.Manual,
                VibrationIntensity = 0f,
                VibrationFrequency = 170f,
                ThermalMode = ThermalModes.Manual,
                ThermalIntensity = -1.0f
            },
            new DotPropsWritable {
                Address = 4,
                LedMode = LedModes.GlobalManual,
                GlobalLed = new RgbLed { Red=0, Green=0, Blue=0 },
                VibrationMode = VibrationModes.Manual,
                VibrationIntensity = 0f,
                VibrationFrequency = 170f,
                ThermalMode = ThermalModes.Manual,
                ThermalIntensity = -1.0f
            },
        };

        try
        {
            using var cts = new CancellationTokenSource(3000);
            var serialClient = new DatafeelModbusClientConfiguration()
                .UseWindowsSerialPortTransceiver()
                .CreateClient();

            bool started = await manager.Start(new List<DatafeelModbusClient> { serialClient }, cts.Token);
            if (!started)
            {
                Console.WriteLine("Could not start DotManager.");
                return;
            }
            _manager = manager;
            Console.WriteLine("DotManager started, writing initial cold/no-vib settings...");

            foreach (var dot in _dots)
            {
                await _manager.Write(dot);
            }
        }
        catch (Exception ex)
        {
            Console.WriteLine("Error starting manager: " + ex.Message);
        }
    }

    // -----------------------------------------------------------------------
    //  Analyze the MP4's audio (with NAudio) => do FFT => if bass>threshold => vibrate
    // -----------------------------------------------------------------------
    private static readonly Queue<float> _energyHistory = new Queue<float>(20);
    private static readonly Queue<float> _fluxHistory = new Queue<float>(20);
    private static float[] _previousSpectrum = new float[fftSize / 2];
    private static float _lastEnergy = 0;

    static void AnalyzeAudioFromMp4(string videoPath)
    {
        try
        {
            using var audioReader = new MediaFoundationReader(videoPath);
            sampleRate = audioReader.WaveFormat.SampleRate;
            Console.WriteLine($"Audio: {audioReader.WaveFormat} (sampleRate={sampleRate})");

            // Convert to Sample Provider for easier float handling
            var sampleProvider = audioReader.ToSampleProvider();

            float[] buffer = new float[fftSize];
            float[] windowedBuffer = new float[fftSize];
            float[] fftMagnitudes = new float[fftSize / 2];

            // For hop size of 1/4 the window
            int hopSize = fftSize / 4;
            float[] overlapBuffer = new float[fftSize];
            int overlapBufferPos = 0;

            // Pre-fill overlap buffer
            sampleProvider.Read(overlapBuffer, 0, fftSize);

            while (!_stopAudio)
            {
                // Shift existing samples left by hop size
                Array.Copy(overlapBuffer, hopSize, overlapBuffer, 0, fftSize - hopSize);

                // Read new samples to fill the gap
                int samplesRead = sampleProvider.Read(overlapBuffer, fftSize - hopSize, hopSize);
                if (samplesRead < hopSize)
                {
                    // End of file or error
                    if (samplesRead <= 0) break;
                }

                // Copy to processing buffer
                Array.Copy(overlapBuffer, buffer, fftSize);

                // Apply window function
                ApplyHannWindow(buffer, windowedBuffer);

                // Compute FFT
                double[] re = new double[fftSize];
                double[] im = new double[fftSize];
                for (int i = 0; i < fftSize; i++)
                {
                    re[i] = windowedBuffer[i];
                    im[i] = 0;
                }

                // Perform FFT
                FFT(re, im);

                // Calculate magnitude spectrum
                for (int i = 0; i < fftSize / 2; i++)
                {
                    fftMagnitudes[i] = (float)Math.Sqrt(re[i] * re[i] + im[i] * im[i]);
                }

                // 1. Calculate RMS energy
                float rmsEnergy = CalculateRMS(buffer);

                // 2. Calculate spectral flux (difference from previous frame)
                float spectralFlux = CalculateSpectralFlux(fftMagnitudes, _previousSpectrum);

                // 3. Update history for adaptive thresholding
                _fluxHistory.Enqueue(spectralFlux);
                if (_fluxHistory.Count > 20) _fluxHistory.Dequeue();

                _energyHistory.Enqueue(rmsEnergy);
                if (_energyHistory.Count > 20) _energyHistory.Dequeue();

                // 4. Bass energy content
                float bassEnergy = SumFrequencyBand(fftMagnitudes, 20, 150);
                float midEnergy = SumFrequencyBand(fftMagnitudes, 150, 2000);
                float bassRatio = bassEnergy / (bassEnergy + midEnergy + 0.0001f);

                // 5. Detect onsets with adaptive threshold
                float meanFlux = _fluxHistory.Average();
                float fluxStdDev = CalculateStdDev(_fluxHistory.ToArray(), meanFlux);
                float adaptiveThreshold = meanFlux + 1.5f * fluxStdDev;

                bool isOnset = spectralFlux > adaptiveThreshold;
                bool isEnergyRising = rmsEnergy > _lastEnergy * 1.3f; // 30% increase
                bool hasSignificantBass = bassRatio > 0.4f; // Bass is 40% of relevant spectrum
                bool isLoudEnough = rmsEnergy > 0.1f; // Minimum volume threshold

                // Final explosion detector
                bool isExplosion = isOnset && isEnergyRising && hasSignificantBass && isLoudEnough;

                if (isExplosion)
                {
                    Console.WriteLine($"EXPLOSION DETECTED! Flux: {spectralFlux:F3}, Energy: {rmsEnergy:F3}, Bass: {bassRatio:P0}");
                    SetDotsVibration(true).Wait();
                    Thread.Sleep(100); // Ensure minimum vibration duration
                }
                else if (_vibrationOn && (DateTime.Now - _lastVibrationOnTime) > _vibrationHold)
                {
                    SetDotsVibration(false).Wait();
                }

                // Save current frame for next comparison
                Array.Copy(fftMagnitudes, _previousSpectrum, fftSize / 2);
                _lastEnergy = rmsEnergy;

                // Sleep less for better responsiveness
                Thread.Sleep(5);
            }
        }
        catch (Exception ex)
        {
            Console.WriteLine("Audio error: " + ex.Message);
        }
    }

    private static void ApplyHannWindow(float[] input, float[] output)
    {
        for (int i = 0; i < input.Length; i++)
        {
            // Hann window: 0.5 * (1 - cos(2π*n/(N-1)))
            double window = 0.5 * (1 - Math.Cos(2 * Math.PI * i / (input.Length - 1)));
            output[i] = input[i] * (float)window;
        }
    }

    private static float CalculateRMS(float[] samples)
    {
        float sum = 0;
        for (int i = 0; i < samples.Length; i++)
        {
            sum += samples[i] * samples[i];
        }
        return (float)Math.Sqrt(sum / samples.Length);
    }

    private static float CalculateSpectralFlux(float[] currentSpectrum, float[] previousSpectrum)
    {
        float sum = 0;
        for (int i = 0; i < currentSpectrum.Length; i++)
        {
            // Only count increases in energy (positive differences)
            float diff = currentSpectrum[i] - previousSpectrum[i];
            if (diff > 0)
                sum += diff;  // Can also use diff*diff for squared difference
        }
        return sum;
    }

    private static float SumFrequencyBand(float[] spectrum, int lowFreq, int highFreq)
    {
        int lowBin = Math.Max(0, (lowFreq * fftSize / sampleRate));
        int highBin = Math.Min(spectrum.Length - 1, (highFreq * fftSize / sampleRate));

        float sum = 0;
        for (int i = lowBin; i <= highBin; i++)
        {
            sum += spectrum[i];
        }
        return sum;
    }

    private static float CalculateStdDev(float[] values, float mean)
    {
        float sumSquaredDiff = 0;
        foreach (float value in values)
        {
            float diff = value - mean;
            sumSquaredDiff += diff * diff;
        }
        return (float)Math.Sqrt(sumSquaredDiff / values.Length);
    }

    // -----------------------------------------------------------------------
    //  We do an FFT on the ring of 1024 samples, sum up bins < 150Hz => amplitude
    // -----------------------------------------------------------------------
    private static float ProcessFFT(float[] samples)
    {
        // Convert float->double, do simple in-place FFT
        double[] re = new double[fftSize];
        double[] im = new double[fftSize];
        for (int i = 0; i < fftSize; i++)
        {
            re[i] = samples[i];
            im[i] = 0.0;
        }

        FFT(re, im);

        // sum magnitude of bins that correspond to freq < 150
        int maxBin = (int)Math.Floor(lowerFreqCutoff * fftSize / (double)sampleRate);
        maxBin = Math.Min(maxBin, fftSize / 2 - 1);

        double sumMag = 0.0;
        for (int bin = 0; bin <= maxBin; bin++)
        {
            double mag = Math.Sqrt(re[bin] * re[bin] + im[bin] * im[bin]);
            sumMag += mag;
        }

        float scaled = (float)(sumMag / (fftSize / 2f));
        return scaled;
    }

    // -----------------------------------------------------------------------
    //  Basic in-place Cooley–Tukey FFT
    // -----------------------------------------------------------------------
    private static void FFT(double[] re, double[] im)
    {
        int n = re.Length;
        // bit-reversal
        int j = 0;
        for (int i = 1; i < n - 1; i++)
        {
            int bit = n >> 1;
            for (; j >= bit; bit >>= 1)
                j -= bit;
            j += bit;
            if (i < j)
            {
                (re[i], re[j]) = (re[j], re[i]);
                (im[i], im[j]) = (im[j], im[i]);
            }
        }
        // cooley-tukey
        for (int len = 2; len <= n; len <<= 1)
        {
            double theta = 2 * Math.PI / len;
            double wtemp = Math.Sin(0.5 * theta);
            double wpr = -2.0 * wtemp * wtemp;
            double wpi = Math.Sin(theta);
            double wr = 1.0;
            double wi = 0.0;
            int halfLen = len >> 1;

            for (int m = 0; m < halfLen; m++)
            {
                for (int i = m; i < n; i += len)
                {
                    int j2 = i + halfLen;
                    double tempr = wr * re[j2] - wi * im[j2];
                    double tempi = wr * im[j2] + wi * re[j2];
                    re[j2] = re[i] - tempr;
                    im[j2] = im[i] - tempi;
                    re[i] += tempr;
                    im[i] += tempi;
                }
                double wtemp2 = wr;
                wr = wtemp2 * wpr - wi * wpi + wr;
                wi = wi * wpr + wtemp2 * wpi + wi;
            }
        }
    }

    // -----------------------------------------------------------------------
    //  Turn vibration on/off
    // -----------------------------------------------------------------------
    private static async Task SetDotsVibration(bool on)
    {
        if (_manager == null || _dots == null) return;
        if (on)
        {
            _vibrationOn = true;
            _lastVibrationOnTime = DateTime.Now;
        }
        else
        {
            _vibrationOn = false;
        }

        foreach (var d in _dots)
        {
            d.VibrationMode = VibrationModes.Manual;
            d.VibrationIntensity = on ? 1.0f : 0.0f;
            try
            {
                await _manager.Write(d);
            }
            catch { }
        }
    }

    // -----------------------------------------------------------------------
    //  Return average color in (R,G,B)
    // -----------------------------------------------------------------------
    private static (byte r, byte g, byte b) GetAverageBGRColor(Mat frame)
    {
        Scalar mean = Cv2.Mean(frame);
        double mb = mean.Val0;
        double mg = mean.Val1;
        double mr = mean.Val2;
        byte b = (byte)Math.Clamp(mb, 0, 255);
        byte g = (byte)Math.Clamp(mg, 0, 255);
        byte r = (byte)Math.Clamp(mr, 0, 255);
        return (r, g, b);
    }

    // -----------------------------------------------------------------------
    //  Set Dot color & thermal
    // -----------------------------------------------------------------------
    private static async Task UpdateDotsColorAndThermal(byte r, byte g, byte b, float thermal)
    {
        if (_dots == null || _manager == null) return;

        foreach (var dot in _dots)
        {
            dot.LedMode = LedModes.GlobalManual;
            dot.GlobalLed.Red = r;
            dot.GlobalLed.Green = g;
            dot.GlobalLed.Blue = b;

            dot.ThermalMode = ThermalModes.Manual;
            dot.ThermalIntensity = thermal;

            try
            {
                await _manager.Write(dot);
            }
            catch { }
        }
    }
}
