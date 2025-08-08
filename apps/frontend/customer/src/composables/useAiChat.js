import { ref } from 'vue'

export const useAiChat = () => {
  const isLoading = ref(false)
  const error = ref(null)

  const sendMessage = async (message) => {
    isLoading.value = true
    error.value = null

    try {
      const response = await fetch(`/api/v1/ai/chat`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          message: message
        })
      })

      if (!response.ok) {
        throw new Error(`HTTP error! status: ${response.status}`)
      }

             const data = await response.json()
       
       // API 응답 구조에 따라 조정
       if (data.result === 'success' && data.data) {
         return data.data
       } else {
         throw new Error('AI 응답을 받지 못했습니다.')
       }
    } catch (err) {
      console.error('AI 채팅 API 호출 실패:', err)
      error.value = err.message
      return '죄송합니다. AI 서비스에 일시적인 문제가 발생했습니다. 잠시 후 다시 시도해주세요.'
    } finally {
      isLoading.value = false
    }
  }

  return {
    sendMessage,
    isLoading,
    error
  }
}
