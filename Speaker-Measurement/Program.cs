using System;
using System.Threading;
using NAudio.Wave;
using NAudio.Wave.SampleProviders;

namespace Speaker_Measurement
{
    internal class Program
    {
        private static float rms = 0;
        private static float rmsdB = 0;
        private static readonly float[] frequencies =
            { 31, 40, 50, 63, 80, 100, 125, 160, 200, 250, 315, 400, 500, 630, 800, 1000, 1250, 1500, 2000, 2500, 3150, 4000, 5000, 6300, 8000, 10000, 12500, 16000 };

        private static readonly float dynamicRange = 48; // Range of the gain in dB

        static void Main(string[] args)
        {
            // Initialize microphone RMS reader
            MicrophoneRMSReader microphoneRMSReader = new MicrophoneRMSReader(48000);
            microphoneRMSReader.OnRMSCalculated += OnRMSCalculated;
            microphoneRMSReader.Start();

            // Step 1: Measure RMS with pink noise
            PinkNoiseProvider pinkNoise = new PinkNoiseProvider();
            var waveOut = new WaveOutEvent();
            waveOut.Init(pinkNoise);
            waveOut.Volume = 0.3f; // Adjust volume level of pink noise
            waveOut.Play();

            Thread.Sleep(1000); // Wait for RMS to stabilize
            float rmsdBInitial = rmsdB; // Store the initial RMS dB level for reference
            waveOut.Stop();
            waveOut.Volume = 1.0f; // Reset volume level

            // Step 2: Measure RMS with silence (baseline noise level)
            Thread.Sleep(1000); // Wait for RMS to stabilize
            float rmsdBSilent = rmsdB; // Baseline noise level

            Console.WriteLine($"Initial RMS dB with Pink Noise: {rmsdBInitial:F2} dB");
            Console.WriteLine($"Silent RMS dB: {rmsdBSilent:F2} dB\n");

            // Step 3: Sweep frequencies and adjust amplitude to match target RMS
            float targetRmsdB = rmsdBInitial; // Target dB level from initial pink noise measurement
            float tolerance = 0.5f; // Tolerance in dB for acceptable match

            foreach (float frequency in frequencies)
            {
                // Console.WriteLine($"\nMeasuring RMS at frequency {frequency} Hz...");

                // Initial gain guess in dB
                float gainDb = 0f;
                int successfulMeasurements = 0; // Count of successful measurements

                // Initialize the sine wave provider once per frequency
                SineWaveProvider32 sineWave = new SineWaveProvider32(frequency, (float)Math.Pow(10, gainDb / 20), 48000);
                waveOut.Init(sineWave);
                waveOut.Play();

                while (successfulMeasurements < 5) // Continue until 5 successful measurements
                {
                    // Convert gain dB to amplitude (linear scale)
                    float amplitude = (float)Math.Pow(10, gainDb / 20);

                    // Set waveOut volume based on the calculated amplitude
                    sineWave.amplitude = amplitude;
                    Thread.Sleep(100); // Wait for RMS to stabilize

                    // Capture the RMS dB at this frequency and amplitude
                    float measuredRmsdB = rmsdB;

                    Console.Write($"\rFrequency: {frequency,6:F0} Hz | Gain: {gainDb,6:F2} dB | Amplitude: {amplitude,6:F2} | RMS dB: {measuredRmsdB,6:F2} dB");

                    // Check if the measured RMS is within the tolerance range
                    float difference = measuredRmsdB - targetRmsdB;

                    if (Math.Abs(difference) <= tolerance)
                    {
                        successfulMeasurements++; // Increment count of successful measurements
                    }
                    else
                    {
                        // Reset successful measurement count on failure
                        successfulMeasurements = 0;

                        // Adjust gain in dB based on whether RMS is too high or too low
                        gainDb += difference > 0 ? -0.5f : 0.5f;

                        // Limit the gain to prevent exceeding the ±12 dB range
                        if (gainDb > dynamicRange)
                        {
                            gainDb = dynamicRange;
                            successfulMeasurements = 5; // Exit loop if gain is out of range
                        }
                        else if (gainDb < -dynamicRange)
                        {
                            gainDb = -dynamicRange;
                            successfulMeasurements = 5; // Exit loop if gain is out of range
                        }
                    }
                }

                // Stop the current wave output after matching RMS
                waveOut.Stop();
                Console.WriteLine();
                //Console.WriteLine($"\nSuccessfully measured {successfulMeasurements} times at {frequency} Hz.");
            }

            microphoneRMSReader.Stop();
        }

        static void OnRMSCalculated(float rms)
        {
            // Convert RMS to dB and update program variables
            Program.rms = rms;
            Program.rmsdB = 20f * MathF.Log10(rms);
        }
    }

    public class SineWaveProvider32 : ISampleProvider
    {
        private readonly WaveFormat waveFormat;
        private readonly double frequency;
        public double amplitude;
        private double phase;
        private readonly double increment;

        public SineWaveProvider32(double frequency, double amplitude, int sampleRate = 48000)
        {
            waveFormat = WaveFormat.CreateIeeeFloatWaveFormat(sampleRate, 1);
            this.frequency = frequency;
            this.amplitude = amplitude;
            increment = 2 * Math.PI * frequency / sampleRate;
        }

        public WaveFormat WaveFormat => waveFormat;

        public int Read(float[] buffer, int offset, int count)
        {
            for (int i = 0; i < count; i++)
            {
                buffer[offset + i] = (float)(amplitude * Math.Sin(phase));
                phase += increment;
                if (phase > 2 * Math.PI) phase -= 2 * Math.PI;
            }
            return count;
        }
    }

    public class PinkNoiseProvider : ISampleProvider
    {
        private readonly WaveFormat waveFormat;
        private readonly Random random;
        private const float Gain = 0.2f; // Adjust volume level of pink noise
        private float[] b = new float[7];

        public PinkNoiseProvider(int sampleRate = 48000)
        {
            waveFormat = WaveFormat.CreateIeeeFloatWaveFormat(sampleRate, 1);
            random = new Random();
        }

        public WaveFormat WaveFormat => waveFormat;

        public int Read(float[] buffer, int offset, int count)
        {
            for (int i = 0; i < count; i++)
            {
                float white = (float)(random.NextDouble() * 2.0 - 1.0);
                b[0] = 0.99886f * b[0] + white * 0.0555179f;
                b[1] = 0.99332f * b[1] + white * 0.0750759f;
                b[2] = 0.96900f * b[2] + white * 0.1538520f;
                b[3] = 0.86650f * b[3] + white * 0.3104856f;
                b[4] = 0.55000f * b[4] + white * 0.5329522f;
                b[5] = -0.7616f * b[5] - white * 0.0168980f;
                float pink = b[0] + b[1] + b[2] + b[3] + b[4] + b[5] + b[6] + white * 0.5362f;
                b[6] = white * 0.115926f;
                buffer[offset + i] = pink * Gain;
            }
            return count;
        }
    }

    public class MicrophoneRMSReader
    {
        private WaveInEvent waveIn;
        private readonly int sampleRate;
        private readonly int channels;

        public event Action<float> OnRMSCalculated;

        public MicrophoneRMSReader(int sampleRate = 44100, int channels = 1)
        {
            this.sampleRate = sampleRate;
            this.channels = channels;

            waveIn = new WaveInEvent
            {
                WaveFormat = new WaveFormat(sampleRate, channels)
            };

            waveIn.DataAvailable += OnDataAvailable;
        }

        public void Start() => waveIn.StartRecording();

        public void Stop() => waveIn.StopRecording();

        private void OnDataAvailable(object sender, WaveInEventArgs e)
        {
            float rms = CalculateRMS(e.Buffer, e.BytesRecorded);
            OnRMSCalculated?.Invoke(rms);
        }

        private float CalculateRMS(byte[] buffer, int bytesRecorded)
        {
            int bytesPerSample = waveIn.WaveFormat.BitsPerSample / 8;
            int sampleCount = bytesRecorded / bytesPerSample;
            float sumSquares = 0;

            for (int i = 0; i < bytesRecorded; i += bytesPerSample)
            {
                float sample = BitConverter.ToInt16(buffer, i) / 32768f;
                sumSquares += sample * sample;
            }

            float meanSquare = sumSquares / sampleCount;
            return (float)Math.Sqrt(meanSquare);
        }
    }
}
