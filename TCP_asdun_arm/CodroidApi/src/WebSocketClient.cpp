#include "WebSocketClient.h"

namespace c2 {
WebSocketClient::WebSocketClient(const std::string& host,
                                 const std::string& port,
                                 std::function<void(std::string)> onRead,
                                 std::function<void()> onClose)
    : _host(host),
      _port(port),
      _resolver(net::make_strand(_ioc)),
      _ws(net::make_strand(_ioc)),
      _receive(onRead),
      _onClosed(onClose) {
}

WebSocketClient::~WebSocketClient() {
    _ioc.stop();
}


}  // namespace c2