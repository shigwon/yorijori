import { ref, onMounted, onUnmounted } from 'vue'
import SockJS from 'sockjs-client'
import { Client } from '@stomp/stompjs'

export const useWebSocket = () => {
  const stompClient = ref(null)
  const connected = ref(false)
  const messages = ref([])
  const orderCode = ref('0DNCG3') // 기본 주문번호

  const connect = () => {
    // SockJS를 통한 WebSocket 연결 (프록시 사용)
    const socket = new SockJS(`/ws/ws-chat`)
    stompClient.value = new Client({
      webSocketFactory: () => socket,
      debug: (str) => {
        console.log('STOMP Debug:', str)
      },
      reconnectDelay: 5000,
      heartbeatIncoming: 4000,
      heartbeatOutgoing: 4000
    })
    
    // STOMP 클라이언트 연결
    stompClient.value.onConnect = (frame) => {
      console.log('STOMP 연결됨:', frame)
      connected.value = true
      
      // 구독 설정
      stompClient.value.subscribe(`/topic/inquiries/${orderCode.value}`, (message) => {
        const receivedMessage = JSON.parse(message.body)
        console.log('메시지 수신:', receivedMessage)
        messages.value.push(receivedMessage)
      })
    }
    
    stompClient.value.onStompError = (frame) => {
      console.error('STOMP 오류:', frame)
      connected.value = false
    }
    
    stompClient.value.onWebSocketError = (error) => {
      console.error('WebSocket 오류:', error)
      connected.value = false
    }
    
    stompClient.value.onWebSocketClose = () => {
      console.log('WebSocket 연결 종료')
      connected.value = false
    }
    
    // 연결 시작
    stompClient.value.activate()
  }

  const disconnect = () => {
    if (stompClient.value) {
      stompClient.value.deactivate()
      connected.value = false
    }
  }

  const sendMessage = (sender, content) => {
    if (!connected.value || !stompClient.value) {
      console.error('STOMP 클라이언트가 연결되지 않음')
      return
    }

    const message = {
      sender: sender,
      content: content,
      timestamp: new Date().toISOString() // ISO 8601 형식 (YYYY-MM-DDTHH:mm:ss.sssZ)
    }

    console.log('메시지 전송:', message)
    
    // STOMP를 통해 메시지 전송
    stompClient.value.publish({
      destination: `/app/inquiry/${orderCode.value}/send`,
      body: JSON.stringify(message)
    })
    
    // 로컬 메시지 목록에 추가
    messages.value.push(message)
  }

  const setOrderCode = (code) => {
    orderCode.value = code
    // 주문번호가 변경되면 재연결
    if (connected.value) {
      disconnect()
      connect()
    }
  }

  onMounted(() => {
    connect()
  })

  onUnmounted(() => {
    disconnect()
  })

  return {
    connected,
    messages,
    orderCode,
    sendMessage,
    setOrderCode,
    connect,
    disconnect
  }
}
