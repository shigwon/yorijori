<template>
  <div class="survey-container">
    <!-- 질문 -->
    <div class="question">{{ questionText }}</div>
    
    <!-- 별점 평가 -->
    <div class="star-rating">
      <div 
        v-for="star in 5" 
        :key="star"
        class="star"
        :class="{ 'filled': star <= selectedRating }"
        @click="selectedRating = star"
      >
        ★
      </div>
    </div>
    
    <!-- 불편한 점 선택 옵션들 -->
    <div class="feedback-options">
      <div class="option-row">
        <button 
          v-for="option in row1Options" 
          :key="option"
          class="option-button"
          :class="{ 'selected': selectedOptions.includes(option) }"
          @click="toggleOption(option)"
        >
          {{ option }}
        </button>
      </div>
      <div class="option-row">
        <button 
          v-for="option in row2Options" 
          :key="option"
          class="option-button"
          :class="{ 'selected': selectedOptions.includes(option) }"
          @click="toggleOption(option)"
        >
          {{ option }}
        </button>
      </div>
      
    </div>
    

    
                   <!-- 추가 피드백 입력 -->
      <div class="feedback-input-section">
        <textarea 
          v-model="additionalFeedback"
          class="feedback-textarea"
          placeholder="불편하셨던 사항을 자유롭게 남겨주세요."
        ></textarea>
      </div>
    
    <!-- 평가 보내기 버튼 -->
    <button class="submit-button" @click="submitSurvey">
      평가 보내기
    </button>
  </div>
</template>

<script setup>
import { ref, computed } from 'vue'
import { useAppState } from '../composables/useAppState'

const selectedRating = ref(1)
const selectedOptions = ref([])
const additionalFeedback = ref('')

// 별점에 따른 옵션 텍스트 변경
const row1Options = computed(() => {
  if (selectedRating.value <= 1) {
    return ['위치가 틀림', '얼굴 인식 실패', '앱 접속 오류']
  } else if (selectedRating.value === 2) {
    return ['위치 정확도 개선', '얼굴 인식 정확도', '앱 안정성 개선']
  } else if (selectedRating.value === 3) {
    return ['위치 서비스 개선', '얼굴 인식 서비스', '앱 사용성 개선']
  } else {
    return ['위치가 틀림', '얼굴 인식 실패', '앱 접속 오류']
  }
})

const row2Options = computed(() => {
  if (selectedRating.value <= 1) {
    return ['내 위치가 안 보임', '너무 느림', '로봇이 바보같음']
  } else if (selectedRating.value === 2) {
    return ['위치 표시 개선', '속도 개선', '로봇 인식 정확도']
  } else if (selectedRating.value === 3) {
    return ['위치 확인 개선', '반응 속도 개선', '로봇 서비스 개선']
  } else {
    return ['내 위치가 안 보임', '너무 느림', '로봇이 바보같음']
  }
})

// 별점에 따른 질문 텍스트 변경
const questionText = computed(() => {
  if (selectedRating.value <= 1) {
    return '어떤 점이 불편하셨나요?'
  } else if (selectedRating.value === 2) {
    return '어떤 점을 개선하면 좋을까요?'
  } else if (selectedRating.value === 4) {
    return '어떤 점이 좋으셨나요?'
  } else if (selectedRating.value === 5) {
    return '어떤 점이 만족스러웠나요?'
  } else {
    return '어떤 점을 개선하면 좋을까요?'
  }
})

const toggleOption = (option) => {
  const index = selectedOptions.value.indexOf(option)
  if (index > -1) {
    selectedOptions.value.splice(index, 1)
    // 텍스트 영역에서 해당 옵션 제거
    additionalFeedback.value = additionalFeedback.value.replace(option + '\n', '')
    additionalFeedback.value = additionalFeedback.value.replace(option, '')
  } else {
    selectedOptions.value.push(option)
    // 텍스트 영역에 해당 옵션 추가
    if (additionalFeedback.value.trim() === '') {
      additionalFeedback.value = option
    } else {
      additionalFeedback.value += '\n' + option
    }
  }
}

// URL에서 주문번호 가져오기
const getOrderCode = () => {
  // Pinia 스토어에서 주문번호 가져오기
  const { orderCode } = useAppState()
  
  console.log('스토어의 orderCode:', orderCode.value)
  
  if (orderCode.value) {
    console.log('스토어에서 주문번호 가져옴:', orderCode.value)
    return orderCode.value
  }
  
  // 스토어에 없으면 URL에서 가져오기 (fallback)
  const urlParams = new URLSearchParams(window.location.search)
  const urlCode = urlParams.get('code')
  console.log('URL 파라미터 code:', urlCode)
  
  // 테스트용 주문번호 설정
  const finalCode = urlCode || '0DNCG3'
  console.log('최종 주문번호:', finalCode)
  return finalCode
}

const submitSurvey = async () => {
  try {
    const orderCode = getOrderCode()
    const reviewData = {
      orderCode: orderCode,
      rating: selectedRating.value,
      content: additionalFeedback.value || null
    }

    console.log('평가 제출:', reviewData)
    console.log('주문번호:', orderCode)
    console.log('API 요청 URL:', '/api/v1/reviews')

    const response = await fetch('/api/v1/reviews', {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
      },
      body: JSON.stringify(reviewData)
    })

    console.log('응답 상태:', response.status)
    console.log('응답 헤더:', response.headers)

    if (response.ok) {
      const responseData = await response.json()
      console.log('평가 제출 성공!', responseData)
      alert('평가가 성공적으로 제출되었습니다!')
      window.close()
    } else {
      const errorText = await response.text()
      console.error('평가 제출 실패:', response.status, errorText)
      alert(`평가 제출에 실패했습니다. (${response.status}) 다시 시도해주세요.`)
    }
  } catch (error) {
    console.error('평가 제출 중 오류 발생:', error)
    console.error('오류 상세:', error.message)
    alert(`평가 제출 중 오류가 발생했습니다: ${error.message}`)
  }
}
</script>

<style scoped>
.survey-container {
  width: 100%;
  height: 100vh;
  height: 100dvh;
  background: white;
  display: flex;
  flex-direction: column;
  align-items: center;
  position: absolute;
  top: 0;
  left: 0;
  right: 0;
  bottom: 0;
  overflow-y: auto;
  z-index: 9999;
  padding: 40px 20px;
}

.question {
  font-size: 18px;
  font-weight: 600;
  color: #333;
  margin-bottom: 20px;
  text-align: center;
}

/* 별점 평가 */
.star-rating {
  display: flex;
  gap: 8px;
  margin-bottom: 30px;
}

.star {
  font-size: 24px;
  color: #ddd;
  cursor: pointer;
  transition: color 0.2s ease;
}

.star.filled {
  color: #7C3AED;
}

/* 불편한 점 선택 옵션들 */
.feedback-options {
  width: 100%;
  max-width: 400px;
  margin-bottom: 30px;
}

.option-row {
  display: flex;
  gap: 8px;
  margin-bottom: 8px;
}

.option-button {
  flex: 1;
  padding: 12px 8px;
  background: #f8f9fa;
  border: 1px solid #e9ecef;
  border-radius: 20px;
  font-size: 12px;
  color: #495057;
  cursor: pointer;
  transition: all 0.2s ease;
  white-space: nowrap;
  overflow: hidden;
  text-overflow: ellipsis;
}

.option-button.selected {
  background: #7C3AED;
  color: white;
  border-color: #7C3AED;
}

.option-button:hover {
  background: #e9ecef;
}

.option-button.selected:hover {
  background: #6D28D9;
}



/* 피드백 입력 섹션 */
.feedback-input-section {
  width: 100%;
  max-width: 400px;
  margin-bottom: 30px;
}



.feedback-textarea {
  width: 100%;
  height: 100px;
  padding: 12px;
  border: 1px solid #ddd;
  border-radius: 8px;
  font-size: 14px;
  resize: none;
  font-family: inherit;
}

.feedback-textarea::placeholder {
  color: #ccc;
  opacity: 0.7;
}

.feedback-textarea:focus {
  outline: none;
  border-color: #7C3AED;
}

/* 제출 버튼 */
.submit-button {
  width: 100%;
  max-width: 400px;
  padding: 16px;
  background: #7C3AED;
  border: none;
  border-radius: 12px;
  font-size: 16px;
  font-weight: 600;
  color: white;
  cursor: pointer;
  transition: all 0.2s ease;
  box-shadow: 0 4px 12px rgba(124, 58, 237, 0.3);
}

.submit-button:hover {
  background: #6D28D9;
  transform: translateY(-2px);
  box-shadow: 0 6px 16px rgba(124, 58, 237, 0.4);
}

.submit-button:active {
  transform: translateY(0);
  box-shadow: 0 2px 8px rgba(124, 58, 237, 0.3);
}

/* 반응형 디자인 */
@media (max-width: 480px) {
  .survey-container {
    padding: 20px 15px;
  }
  
  .question {
    font-size: 16px;
    margin-bottom: 15px;
  }
  
  .star {
    font-size: 20px;
  }
  
  .option-button {
    padding: 10px 6px;
    font-size: 11px;
  }
  
  .feedback-textarea {
    height: 80px;
  }
}
</style> 