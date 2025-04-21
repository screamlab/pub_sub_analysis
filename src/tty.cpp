#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdio>  // for popen, pclose
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

using namespace std;

// Global control flag (set to false on SIGINT)
atomic<bool> running(true);

// Device path and package size (set via command-line arguments)
string devicePath;
int packageSize = 0;  // number of values per package

// Mutex to protect shared statistics
mutex mtx;

// Global statistics for package loss tracking.
int first_seq = -1;        // first package sequence seen
int last_seq = -1;         // last package sequence seen
int overall_received = 0;  // count of packages received
int overall_loss = 0;      // accumulated lost packages

// Per-second statistics.
int current_sec = 0;       // current epoch second used to accumulate per-second stats
int sec_expected = 0;      // expected package count in the current second (sum of differences)
int sec_received = 0;      // number of packages received in the current second
int sec_loss = 0;          // lost packages in the current second
vector<int> loss_per_sec;  // history: lost packages per finished second
vector<int> receive_rate_per_sec;  // history: received packages per finished second

// A struct to hold plot data.
struct PlotData {
    double time;         // elapsed time (seconds) relative to start
    double lossPercent;  // current loss percentage in this second
    double receiveRate;  // current receive rate in Hz for this second
};
vector<PlotData> plotData;

// Start time for overall rate calculation.
chrono::steady_clock::time_point start_time;

// Print usage information.
void printUsage(const char *progName) {
    cout << "Usage: " << progName << " -d <serial_device_path> -s <package_size>\n\n"
         << "Options:\n"
         << "  -d, --device   [DEVICE]     Specify the serial device path (e.g., /dev/ttyUSB0).\n"
         << "  -s, --size     [SIZE]       Specify the size of each package (e.g., 11).\n"
         << "  -h, --help                 Print this help message and exit.\n";
}

// SIGINT handler – simply mark the running flag false.
void sigintHandler(int signum) {
    signum = signum;  // avoid unused parameter warning
    running = false;
}

// Helper function to read a full line (ending with '\n') from the serial device.
// It uses the file descriptor and an accumulating string buffer.
string readLine(int fd, string &buffer) {
    const int bufsize = 256;
    char temp[bufsize];
    while (running) {
        size_t pos = buffer.find('\n');
        if (pos != string::npos) {
            string line = buffer.substr(0, pos);
            buffer.erase(0, pos + 1);
            return line;
        }
        int n = read(fd, temp, bufsize);
        if (n > 0) {
            buffer.append(temp, n);
        } else {
            // No data available; wait a bit.
            this_thread::sleep_for(chrono::milliseconds(10));
        }
    }
    return "";
}

// This thread function reads serial data from the device.
void readSerial() {
    // Open the device in read/write mode without making it the controlling terminal.
    int fd = open(devicePath.c_str(), O_RDWR | O_NOCTTY);
    if (fd == -1) {
        cerr << "Error: Failed to open " << devicePath << endl;
        running = false;
        return;
    }

    // Set file descriptor to blocking mode.
    fcntl(fd, F_SETFL, 0);

    // Configure serial port settings.
    struct termios options;
    if (tcgetattr(fd, &options) < 0) {
        cerr << "Error: tcgetattr() failed" << endl;
        close(fd);
        running = false;
        return;
    }

    // Set input and output baud rate to 921600.
    cfsetispeed(&options, B921600);
    cfsetospeed(&options, B921600);

    // Configure 8N1: 8 data bits, no parity, 1 stop bit.
    options.c_cflag &= ~PARENB;         // disable parity
    options.c_cflag &= ~CSTOPB;         // 1 stop bit
    options.c_cflag &= ~CSIZE;          // clear current data size setting
    options.c_cflag |= CS8;             // 8 data bits
    options.c_cflag |= CREAD | CLOCAL;  // enable receiver, ignore modem control lines

    // Set raw mode (non-canonical, no echo, no signals)
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    options.c_iflag &= ~(IXON | IXOFF | IXANY);  // disable software flow control
    options.c_oflag &= ~OPOST;                   // disable output processing

    // Apply the settings immediately.
    tcsetattr(fd, TCSANOW, &options);

    string dataBuffer;
    while (running) {
        // Read a full line from the serial port.
        string line = readLine(fd, dataBuffer);
        if (!line.empty()) {
            // Check if this line is a header line (e.g., "#251:")
            if (line[0] == '#') {
                int numbersRead = 0;
                double packageValue = 0.0;
                bool gotValue = false;

                // Read subsequent lines until we have obtained packageSize numbers.
                while (numbersRead < packageSize && running) {
                    string numsLine = readLine(fd, dataBuffer);
                    istringstream iss(numsLine);
                    double val;
                    while (iss >> val) {
                        if (!gotValue) {
                            packageValue = val;
                            gotValue = true;
                        }
                        numbersRead++;
                        if (numbersRead >= packageSize)
                            break;
                    }
                }

                int seq = static_cast<int>(packageValue);
                // Get the current epoch second.
                int now_sec =
                    static_cast<int>(chrono::system_clock::to_time_t(chrono::system_clock::now()));

                {
                    lock_guard<mutex> lock(mtx);
                    // Check if we have crossed a second boundary BEFORE updating per-second
                    // counters.
                    if (current_sec == 0) {
                        current_sec = now_sec;
                    } else if (now_sec != current_sec) {
                        // Finalize the previous second.
                        loss_per_sec.push_back(sec_loss);
                        receive_rate_per_sec.push_back(sec_received);
                        // Also record data for plotting.
                        double currentTime =
                            chrono::duration<double>(chrono::steady_clock::now() - start_time)
                                .count();
                        double curLossPercent =
                            (sec_expected > 0) ? (100.0 * sec_loss / sec_expected) : 0.0;
                        PlotData pd{currentTime, curLossPercent, static_cast<double>(sec_received)};
                        plotData.push_back(pd);

                        // Reset per-second counters for the new second.
                        sec_expected = 0;
                        sec_received = 0;
                        sec_loss = 0;
                        current_sec = now_sec;
                    }

                    // Process packet sequence.
                    if (first_seq < 0) {
                        first_seq = seq;
                        last_seq = seq;
                        overall_received = 1;
                    } else {
                        int diff = seq - last_seq;
                        int lost = (diff > 1) ? (diff - 1) : 0;

                        overall_received++;    // one package received
                        overall_loss += lost;  // update overall lost count

                        // Update per-second counters for this package.
                        sec_expected += diff;
                        sec_received++;
                        sec_loss += lost;

                        last_seq = seq;
                    }
                }
            }
        }
    }
    close(fd);
}

// This thread function prints statistics every second.
void printStats() {
    while (running) {
        this_thread::sleep_for(chrono::seconds(1));
        lock_guard<mutex> lock(mtx);

        // Compute current loss percentage.
        double curLossPercent = (sec_expected > 0) ? (100.0 * sec_loss / sec_expected) : 0.0;
        int overall_expected = overall_received + overall_loss;
        double overallLossPercent =
            (overall_expected > 0) ? (100.0 * overall_loss / overall_expected) : 0.0;

        // Calculate total elapsed time for overall receive rate.
        double elapsed = chrono::duration_cast<chrono::duration<double>>(
                             chrono::steady_clock::now() - start_time)
                             .count();
        double total_receive_rate = (elapsed > 0) ? (overall_received / elapsed) : 0.0;

        // Compute standard deviation of loss/sec.
        double lossSum = 0;
        for (int loss : loss_per_sec)
            lossSum += loss;
        double lossMean = loss_per_sec.empty() ? 0.0 : lossSum / loss_per_sec.size();
        double lossVariance = 0;
        for (int loss : loss_per_sec)
            lossVariance += (loss - lossMean) * (loss - lossMean);
        double stdevLoss = loss_per_sec.empty() ? 0.0 : sqrt(lossVariance / loss_per_sec.size());

        // Compute standard deviation of receive rates.
        double recvSum = 0;
        for (int rate : receive_rate_per_sec)
            recvSum += rate;
        double recvMean =
            receive_rate_per_sec.empty() ? 0.0 : recvSum / receive_rate_per_sec.size();
        double recvVariance = 0;
        for (int rate : receive_rate_per_sec)
            recvVariance += (rate - recvMean) * (rate - recvMean);
        double stdevRecv =
            receive_rate_per_sec.empty() ? 0.0 : sqrt(recvVariance / receive_rate_per_sec.size());

        cout << "========================================" << endl;
        cout << "Current Loss/sec: " << sec_loss << ", Current Loss Percentage: " << curLossPercent
             << " %" << endl;
        cout << "Total Data Count: " << overall_expected << ", Total Loss Count: " << overall_loss
             << ", Total Loss Percentage: " << overallLossPercent << " %" << endl;
        cout << "stdev Loss/sec: " << stdevLoss << endl;
        cout << "Current receive rate: " << sec_received << " Hz" << endl;
        cout << "Total receive rate: " << total_receive_rate << " Hz" << endl;
        cout << "stdev receive rate: " << stdevRecv << endl;
    }
}

// Write the collected plot data to a file and use GNUplot to plot it.
void plotUsingGnuplot() {
    // Write data to file "plot_data.dat". Each line: <time> <loss_percentage> <receive_rate>
    ofstream ofs("plot_data.dat");
    if (!ofs.is_open()) {
        cerr << "Error: Could not open plot_data.dat for writing" << endl;
        return;
    }
    // Compute max receive rate from our in‐memory data :contentReference[oaicite:0]{index=0}
    double maxRate = 0.0;
    for (const auto &pd : plotData) {
        ofs << pd.time << " " << pd.lossPercent << " " << pd.receiveRate << "\n";
        if (pd.receiveRate > maxRate) {
            maxRate = pd.receiveRate;
        }
    }
    ofs.close();

    // Now open a pipe to GNUplot.
    FILE *gp = popen("gnuplot -persistent", "w");
    if (gp == nullptr) {
        cerr << "Error: Could not open pipe to gnuplot" << endl;
        return;
    }

    // Generate the GNUplot commands. We set up dual y-axes.
    fprintf(gp, "set title 'Current Loss Percentage and Receive Rate vs Time'\n");
    fprintf(gp, "set xlabel 'Time (s)'\n");
    fprintf(gp, "set ylabel 'Current Loss Percentage %%'\n");
    fprintf(gp, "set yrange [-5:100]\n");  // force loss% between 0 and 100
                                           // :contentReference[oaicite:2]{index=2}
    fprintf(gp, "set y2label 'Current Receive Rate (Hz)'\n");
    fprintf(gp, "set y2tics\n");
    // Use our C++ maxRate (add 10% headroom) :contentReference[oaicite:1]{index=1}
    fprintf(gp, "set y2range [0:%d]\n", int(maxRate * 1.1));
    fprintf(gp, "set grid\n");
    fprintf(gp, "plot 'plot_data.dat' using 1:2 with lines title 'Loss %%', \\\n");
    fprintf(gp, "     'plot_data.dat' using 1:3 axes x1y2 with lines title 'Receive Rate (Hz)'\n");
    fflush(gp);
    pclose(gp);
}

// After SIGINT (or when running stops), print final overall statistics and plot the collected data.
void printFinalStats() {
    lock_guard<mutex> lock(mtx);
    // Finalize the last second if unfinished.
    if (sec_received > 0) {
        loss_per_sec.push_back(sec_loss);
        receive_rate_per_sec.push_back(sec_received);
        double currentTime =
            chrono::duration<double>(chrono::steady_clock::now() - start_time).count();
        double curLossPercent = (sec_expected > 0) ? (100.0 * sec_loss / sec_expected) : 0.0;
        PlotData pd{currentTime, curLossPercent, static_cast<double>(sec_received)};
        plotData.push_back(pd);
    }

    double elapsed =
        chrono::duration_cast<chrono::duration<double>>(chrono::steady_clock::now() - start_time)
            .count();
    int overall_expected = overall_received + overall_loss;
    double overallLossPercent =
        (overall_expected > 0) ? (100.0 * overall_loss / overall_expected) : 0.0;
    double overall_receive_rate = (elapsed > 0) ? (overall_received / elapsed) : 0.0;

    // Compute stdev for loss/sec.
    double lossSum = 0;
    for (int loss : loss_per_sec)
        lossSum += loss;
    double lossMean = loss_per_sec.empty() ? 0.0 : lossSum / loss_per_sec.size();
    double lossVariance = 0;
    for (int loss : loss_per_sec)
        lossVariance += (loss - lossMean) * (loss - lossMean);
    double stdevLoss = loss_per_sec.empty() ? 0.0 : sqrt(lossVariance / loss_per_sec.size());

    // Compute stdev for receive rate.
    double recvSum = 0;
    for (int rate : receive_rate_per_sec)
        recvSum += rate;
    double recvMean = receive_rate_per_sec.empty() ? 0.0 : recvSum / receive_rate_per_sec.size();
    double recvVariance = 0;
    for (int rate : receive_rate_per_sec)
        recvVariance += (rate - recvMean) * (rate - recvMean);
    double stdevRecv =
        receive_rate_per_sec.empty() ? 0.0 : sqrt(recvVariance / receive_rate_per_sec.size());

    cout << "\n########### FINAL STATISTICS ###########\n";
    cout << "Overall Data Count: " << overall_expected << endl;
    cout << "Overall Loss Count: " << overall_loss << endl;
    cout << "Overall Loss Percentage: " << overallLossPercent << " %" << endl;
    cout << "Overall stdev Loss/sec: " << stdevLoss << endl;
    cout << "Overall receive rate: " << overall_receive_rate << " Hz" << endl;
    cout << "Overall stdev receive rate: " << stdevRecv << endl;

    // Plot the current loss percentage and receive rate versus time.
    plotUsingGnuplot();
}

int main(int argc, char *argv[]) {
    if (argc < 5) {
        printUsage(argv[0]);
        return 1;
    }

    // Parse command-line arguments.
    for (int i = 1; i < argc; i++) {
        string arg = argv[i];
        if (arg == "-h" || arg == "--help") {
            printUsage(argv[0]);
            return 0;
        } else if (arg == "-d" || arg == "--device") {
            if (i + 1 < argc) {
                devicePath = argv[++i];
            } else {
                cerr << "Error: Missing argument for " << arg << endl;
                return 1;
            }
        } else if (arg == "-s" || arg == "--size") {
            if (i + 1 < argc) {
                packageSize = atoi(argv[++i]);
                if (packageSize <= 0) {
                    cerr << "Error: Package size must be a positive integer." << endl;
                    return 1;
                }
            } else {
                cerr << "Error: Missing argument for " << arg << endl;
                return 1;
            }
        } else {
            cerr << "Unknown option: " << arg << endl;
            printUsage(argv[0]);
            return 1;
        }
    }

    if (devicePath.empty() || packageSize <= 0) {
        cerr << "Error: Both device and package size must be provided." << endl;
        printUsage(argv[0]);
        return 1;
    }

    // Set the start time for overall receive rate calculation.
    start_time = chrono::steady_clock::now();

    // Set up the SIGINT (Ctrl-C) handler.
    signal(SIGINT, sigintHandler);

    // Start threads for reading serial data and printing statistics.
    thread reader(readSerial);
    thread printer(printStats);

    reader.join();
    printer.join();

    // Print final statistics and plot the data.
    printFinalStats();

    return 0;
}
