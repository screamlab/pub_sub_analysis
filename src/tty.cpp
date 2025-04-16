#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdlib>
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

// Global statistics
int first_seq = -1;        // first package sequence seen
int last_seq = -1;         // last package sequence seen
int overall_received = 0;  // count of logs processed (each log equals one package received)
int overall_loss = 0;      // accumulated lost packages

// For per-second stats
int current_sec = 0;       // current second (epoch time) for which we are accumulating
int sec_expected = 0;      // expected package count in the current second (sum of differences)
int sec_received = 0;      // number of logs received in current second
int sec_loss = 0;          // lost packages in current second
vector<int> loss_per_sec;  // history (each element = lost packages in a finished second)


// Print usage information.
void printUsage(const char *progName) {
    cout << "Usage: " << progName << " -d <serial_device_path> -s <package_size>\n\n"
         << "Options:\n"
         << "  -d, --device   [DEVICE]     Specify the serial device path (e.g., /dev/ttyUSB0).\n"
         << "  -s, --size     [SIZE]       Specify the size of each package (e.g., 11).\n"
         << "  -h, --help                  Print this help message and exit.\n";
}

// SIGINT handler – simply mark the running flag false.
void sigintHandler(int signum) {
    signum = signum;  // avoid unused parameter warning
    running = false;
}

// Helper function to read a full line (ending with '\n') from the serial device.
// It uses the provided file descriptor and a reference buffer to accumulate partial data.
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
    // Open device in read/write mode without making it the controlling terminal.
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
                    // Check the time boundary BEFORE updating per-second counters.
                    if (current_sec == 0) {
                        current_sec = now_sec;
                    } else if (now_sec != current_sec) {
                        // Finalize previous second's statistics.
                        loss_per_sec.push_back(sec_loss);
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
        double curLossPercent = (sec_expected > 0) ? (100.0 * sec_loss / sec_expected) : 0.0;
        int overall_expected = overall_received + overall_loss;
        double overallLossPercent =
            (overall_expected > 0) ? (100.0 * overall_loss / overall_expected) : 0.0;

        // Compute standard deviation of loss/sec from finished seconds.
        double sum = 0;
        for (int loss : loss_per_sec)
            sum += loss;
        double mean = loss_per_sec.empty() ? 0.0 : sum / loss_per_sec.size();
        double variance = 0;
        for (int loss : loss_per_sec)
            variance += (loss - mean) * (loss - mean);
        double stdev = loss_per_sec.empty() ? 0.0 : sqrt(variance / loss_per_sec.size());

        cout << "========================================" << endl;
        cout << "Current Loss/sec: " << sec_loss << ", Current Loss Percentage: " << curLossPercent
             << " %" << endl;
        cout << "Total Data Count: " << overall_expected << ", Total Loss Count: " << overall_loss
             << ", Total Loss Percentage: " << overallLossPercent << " %" << endl;
        cout << "stdev Loss/sec: " << stdev << endl;
    }
}

// After SIGINT (or when running stops), print overall statistics.
void printFinalStats() {
    lock_guard<mutex> lock(mtx);
    // If there is an unfinished second, record its loss value.
    if (sec_received > 0) {
        loss_per_sec.push_back(sec_loss);
    }
    int overall_expected = overall_received + overall_loss;
    double overallLossPercent =
        (overall_expected > 0) ? (100.0 * overall_loss / overall_expected) : 0.0;

    double sum = 0;
    for (int loss : loss_per_sec)
        sum += loss;
    double mean = loss_per_sec.empty() ? 0.0 : sum / loss_per_sec.size();
    double variance = 0;
    for (int loss : loss_per_sec)
        variance += (loss - mean) * (loss - mean);
    double stdev = loss_per_sec.empty() ? 0.0 : sqrt(variance / loss_per_sec.size());

    cout << "\n########### FINAL STATISTICS ###########\n";
    cout << "Overall Data Count: " << overall_expected << endl;
    cout << "Overall Loss Count: " << overall_loss << endl;
    cout << "Overall Loss Percentage: " << overallLossPercent << " %" << endl;
    cout << "Overall stdev Loss/sec: " << stdev << endl;
}

int main(int argc, char *argv[]) {
    if (argc < 5) {
        printUsage(argv[0]);
        return 1;
    }

    // Simple command-line argument parsing.
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

    // Set up the SIGINT (Ctrl-C) handler.
    signal(SIGINT, sigintHandler);

    // Start threads for reading serial data and printing statistics.
    thread reader(readSerial);
    thread printer(printStats);

    reader.join();
    printer.join();

    // Print final statistics.
    printFinalStats();

    return 0;
}
