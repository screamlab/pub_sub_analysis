#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdio>  // popen, pclose
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <numeric>  // accumulate
#include <sstream>
#include <string>
#include <thread>
#include <vector>

using namespace std;

// Buffer size for reading from the serial device.
const size_t bufsize = 256;
string logFileName = "log.csv";

// Global control flag (set to false on SIGINT)
atomic<bool> running(true);

// Device path and package size (set via command-line arguments)
string devicePath;

// Mutex to protect shared statistics
mutex mtx;

// For computing rates & losses
int initial_content = 0;
int last_content = 0;
int last_pkg_cnt = -1;
bool initial_set = true;

int total_recv_pkgs = 0;
int total_lost_pkgs = 0;
int sec_recv_pkgs = 0;
int sec_lost_pkgs = 0;

// History of per-second stats (for avg & stdev)
vector<double> lossPercHistory;
vector<int> recvRateHistory;

// Start time for overall rate calculation.
chrono::steady_clock::time_point start_time;

// Print usage information.
void printUsage(const char *progName) {
    cout << "Usage: " << progName << " -d <serial_device_path> [-l <log_file>]\n\n"
         << "Options:\n"
         << "  -d, --device   [DEVICE]     Specify the serial device path (e.g., /dev/ttyUSB0).\n"
         << "  -l, --log      [LOG_FILE]   Specify the CSV log file path (default: " << logFileName
         << ").\n"
         << "  -h, --help                  Print this help message and exit.\n";
}

// SIGINT handler – simply mark the running flag false.
void sigintHandler(int signum) {
    signum = signum;  // avoid unused parameter warning
    running = false;
}

// Helper function to read a full line (ending with '\n') from the serial device.
// It uses the file descriptor and an accumulating string buffer.
string readLine(int fd, string &buffer) {
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
        if (line.empty())
            continue;

        // Expected format: "#<pkg_cnt>: <val> <val> ... "
        istringstream iss(line);
        string hdr;
        iss >> hdr;
        if (hdr.size() < 3 || hdr.front() != '#' || hdr.back() != ':')
            continue;
        int pkg_cnt = stoi(hdr.substr(1, hdr.size() - 2));

        // filter out duplicates of the same package ID
        if (pkg_cnt == last_pkg_cnt) {
            continue;
        }

        // read the first value in the package; all are identical
        int content;
        iss >> content;

        int loss = 0;
        if (!initial_set) {
            int diff = content - last_content;
            if (diff > 1) {
                loss = diff - 1;
            }
        }

        // update shared stats
        {
            lock_guard<mutex> lk(mtx);
            if (initial_set) {
                initial_content = content;
                initial_set = false;
            }
            last_content = content;
            last_pkg_cnt = pkg_cnt;
            total_recv_pkgs++;
            sec_recv_pkgs++;
            if (loss > 0) {
                total_lost_pkgs += loss;
                sec_lost_pkgs += loss;
            }
        }
    }
    close(fd);
}

// Thread that every second computes & prints/logs the 9 required metrics.
void statsThread() {
    ofstream logFile(logFileName, ios::app);
    if (!logFile) {
        cerr << "Error: cannot open log.csv for writing\n";
        running = false;
        return;
    }

    // Write CSV header if file is empty
    logFile.seekp(0, ios::end);
    if (logFile.tellp() == 0) {
        logFile << "Overall data amount," << "Overall package loss," << "Receive rate (Hz),"
                << "Package loss this second," << "Loss percentage this second,"
                << "Average loss percentage," << "Loss percentage stdev,"
                << "Average receive rate (Hz)," << "Receive rate stdev\n";
    }

    while (running) {
        this_thread::sleep_for(chrono::seconds(1));

        int recv_sec, lost_sec, total_lost, last_cn, init_cn;
        {
            lock_guard<mutex> lk(mtx);
            recv_sec = sec_recv_pkgs;
            lost_sec = sec_lost_pkgs;
            total_lost = total_lost_pkgs;
            last_cn = last_content;
            init_cn = initial_content;
            sec_recv_pkgs = 0;
            sec_lost_pkgs = 0;
        }

        int content_diff = last_cn - init_cn;
        int overall_data_amount = content_diff;

        double loss_pct_sec = 0.0;
        if (recv_sec + lost_sec > 0) {
            loss_pct_sec = (double)lost_sec / (recv_sec + lost_sec) * 100.0;
        }

        lossPercHistory.push_back(loss_pct_sec);
        recvRateHistory.push_back(recv_sec);

        double avg_loss_pct = accumulate(lossPercHistory.begin(), lossPercHistory.end(), 0.0) /
                              lossPercHistory.size();
        double sumsq = 0;
        for (double v : lossPercHistory) {
            sumsq += (v - avg_loss_pct) * (v - avg_loss_pct);
        }
        double stdev_loss_pct = sqrt(sumsq / lossPercHistory.size());

        double avg_recv_rate = accumulate(recvRateHistory.begin(), recvRateHistory.end(), 0.0) /
                               recvRateHistory.size();
        sumsq = 0;
        for (int v : recvRateHistory) {
            sumsq += (v - avg_recv_rate) * (v - avg_recv_rate);
        }
        double stdev_recv_rate = sqrt(sumsq / recvRateHistory.size());

        // 1) Human-readable block to stdout and old log.dat (if you still want it)
        ostringstream human;
        human << "Overall data amount: " << overall_data_amount << "\n"
              << "Overall package loss: " << total_lost << "\n"
              << "Receive rate: " << recv_sec << " Hz" << "\n"
              << "Package loss this second: " << lost_sec << "\n"
              << "Loss percentage this second: " << loss_pct_sec << " %\n"
              << "Average loss percentage: " << avg_loss_pct << " %\n"
              << "Loss percentage stdev: " << stdev_loss_pct << " %\n"
              << "Average receive rate: " << avg_recv_rate << " Hz\n"
              << "Receive rate stdev: " << stdev_recv_rate << " Hz\n\n";

        string block = human.str();
        cout << block;
        // (If you still want the old .dat file, uncomment:)
        // ofstream old("log.dat", ios::app); old<<block; old.close();

        // 2) CSV row to log.csv
        logFile << overall_data_amount << ',' << total_lost << ',' << recv_sec << ',' << lost_sec
                << ',' << fixed << setprecision(6) << loss_pct_sec << ',' << avg_loss_pct << ','
                << stdev_loss_pct << ',' << avg_recv_rate << ',' << stdev_recv_rate << '\n';
        logFile.flush();
    }

    logFile.close();
}

int main(int argc, char *argv[]) {
    if (argc < 3) {
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
        } else if (arg == "-l" || arg == "--log") {
            if (i + 1 < argc) {
                logFileName = argv[++i];
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

    if (devicePath.empty()) {
        cerr << "Error: Device path must be provided." << endl;
        printUsage(argv[0]);
        return 1;
    }

    // Set the start time for overall receive rate calculation.
    start_time = chrono::steady_clock::now();

    // Set up the SIGINT (Ctrl-C) handler.
    signal(SIGINT, sigintHandler);

    // Start threads for reading serial data and printing statistics.
    thread reader(readSerial);
    thread stats(statsThread);

    reader.join();
    stats.join();

    return 0;
}
