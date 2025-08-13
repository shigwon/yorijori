<template>
  <div class="qr-generator-container">
    <!-- 헤더 섹션 -->
    <div class="header-section">
      <h1 class="title">로봇 QR 코드 생성</h1>
      <p class="subtitle">로봇 ID를 입력하면 how-to-use 화면으로 이동하는 QR 코드가 생성됩니다</p>
    </div>

    <!-- 입력 섹션 -->
    <div class="input-section">
      <div class="input-group">
        <label for="robotId">로봇 ID:</label>
        <input 
          id="robotId" 
          v-model="robotId" 
          type="number" 
          placeholder="1"
          min="1"
          class="robot-input"
        />
      </div>
      
      <button @click="generateQRCode" :disabled="!robotId || isGenerating" class="generate-btn">
        {{ isGenerating ? '생성 중...' : 'QR 코드 생성' }}
      </button>
    </div>

    <!-- QR 코드 표시 섹션 -->
    <div v-if="qrCodeDataUrl" class="qr-display-section">
      <h3>생성된 QR 코드</h3>
      <div class="qr-code-container">
        <img :src="qrCodeDataUrl" alt="Generated QR Code" class="qr-code-image" />
      </div>
      
      <div class="qr-info">
        <p><strong>로봇 ID:</strong> {{ robotId }}</p>
        <p><strong>이동할 화면:</strong> How To Use</p>
        <p><strong>URL:</strong> {{ generatedUrl }}</p>
      </div>
      
      <div class="action-buttons">
        <button @click="downloadQRCode" class="download-btn">QR 코드 다운로드</button>
        <button @click="copyUrl" class="copy-btn">URL 복사</button>
        <button @click="resetQR" class="reset-btn">새로 만들기</button>
      </div>
    </div>

    <!-- 테스트 안내 -->
    <div v-if="qrCodeDataUrl" class="test-guide">
      <h4>테스트 방법</h4>
      <ol>
        <li>생성된 QR 코드를 스마트폰으로 스캔하세요</li>
        <li>rider 앱의 how-to-use 화면으로 자동 이동됩니다</li>
        <li>로봇 ID가 URL 파라미터로 전달됩니다</li>
      </ol>
    </div>
  </div>
</template>

<script setup>
import { ref } from 'vue'
import QRCode from 'qrcode'

const robotId = ref('')
const qrCodeDataUrl = ref('')
const generatedUrl = ref('')
const isGenerating = ref(false)

const generateQRCode = async () => {
  if (!robotId.value) return
  
  isGenerating.value = true
  
  try {
    // how-to-use 화면으로 이동하는 URL 생성
    const baseUrl = window.location.origin
    generatedUrl.value = `${baseUrl}/rider/how-to-use?robotId=${robotId.value}`
    
    // QR 코드 생성
    qrCodeDataUrl.value = await QRCode.toDataURL(generatedUrl.value, {
      width: 300,
      margin: 2,
      color: {
        dark: '#000000',
        light: '#FFFFFF'
      }
    })
    
    console.log('QR 코드 생성 완료:', generatedUrl.value)
  } catch (error) {
    console.error('QR 코드 생성 실패:', error)
    alert('QR 코드 생성에 실패했습니다.')
  } finally {
    isGenerating.value = false
  }
}

const downloadQRCode = () => {
  if (!qrCodeDataUrl.value) return
  
  const link = document.createElement('a')
  link.download = `robot-${robotId.value}-howtouse-qr.png`
  link.href = qrCodeDataUrl.value
  link.click()
}

const copyUrl = async () => {
  if (!generatedUrl.value) return
  
  try {
    await navigator.clipboard.writeText(generatedUrl.value)
    alert('URL이 클립보드에 복사되었습니다!')
  } catch (error) {
    console.error('URL 복사 실패:', error)
    alert('URL 복사에 실패했습니다.')
  }
}

const resetQR = () => {
  qrCodeDataUrl.value = ''
  generatedUrl.value = ''
  robotId.value = ''
}
</script>

<style scoped>
.qr-generator-container {
  min-height: 100vh;
  width: 100%;
  background: white;
  display: flex;
  flex-direction: column;
  padding: 20px 24px 24px 24px;
  box-sizing: border-box;
}

/* 헤더 섹션 */
.header-section {
  text-align: center;
  margin-bottom: 40px;
  margin-top: 40px;
}

.title {
  font-size: 24px;
  font-weight: 700;
  color: #1F2937;
  margin: 0 0 8px 0;
  line-height: 1.3;
}

.subtitle {
  font-size: 16px;
  color: #6B7280;
  margin: 0;
  line-height: 1.5;
}

/* 입력 섹션 */
.input-section {
  display: flex;
  flex-direction: column;
  gap: 20px;
  margin-bottom: 40px;
  align-items: center;
}

.input-group {
  display: flex;
  flex-direction: column;
  gap: 8px;
  width: 100%;
  max-width: 300px;
}

.input-group label {
  font-size: 16px;
  font-weight: 600;
  color: #1F2937;
}

.robot-input {
  padding: 12px 16px;
  border: 2px solid #E5E7EB;
  border-radius: 8px;
  font-size: 16px;
  transition: border-color 0.2s ease;
}

.robot-input:focus {
  outline: none;
  border-color: #8B5CF6;
}

.generate-btn {
  padding: 12px 24px;
  background: #8B5CF6;
  color: white;
  border: none;
  border-radius: 8px;
  font-size: 16px;
  font-weight: 600;
  cursor: pointer;
  transition: background-color 0.2s ease;
  min-width: 200px;
}

.generate-btn:hover:not(:disabled) {
  background: #7C3AED;
}

.generate-btn:disabled {
  background: #9CA3AF;
  cursor: not-allowed;
}

/* QR 코드 표시 섹션 */
.qr-display-section {
  display: flex;
  flex-direction: column;
  align-items: center;
  gap: 20px;
  margin-bottom: 40px;
}

.qr-display-section h3 {
  font-size: 20px;
  font-weight: 600;
  color: #1F2937;
  margin: 0;
}

.qr-code-container {
  border: 2px solid #E5E7EB;
  border-radius: 12px;
  padding: 20px;
  background: white;
}

.qr-code-image {
  width: 300px;
  height: 300px;
  display: block;
}

.qr-info {
  text-align: center;
  background: #F9FAFB;
  padding: 16px;
  border-radius: 8px;
  min-width: 300px;
}

.qr-info p {
  margin: 8px 0;
  font-size: 14px;
  color: #374151;
}

.qr-info strong {
  color: #1F2937;
}

/* 액션 버튼들 */
.action-buttons {
  display: flex;
  gap: 12px;
  flex-wrap: wrap;
  justify-content: center;
}

.download-btn, .copy-btn, .reset-btn {
  padding: 10px 20px;
  border: none;
  border-radius: 6px;
  font-size: 14px;
  font-weight: 600;
  cursor: pointer;
  transition: all 0.2s ease;
}

.download-btn {
  background: #10B981;
  color: white;
}

.download-btn:hover {
  background: #059669;
}

.copy-btn {
  background: #3B82F6;
  color: white;
}

.copy-btn:hover {
  background: #2563EB;
}

.reset-btn {
  background: #6B7280;
  color: white;
}

.reset-btn:hover {
  background: #4B5563;
}

/* 테스트 가이드 */
.test-guide {
  background: #FEF3C7;
  border: 1px solid #F59E0B;
  border-radius: 8px;
  padding: 20px;
  margin-top: 20px;
}

.test-guide h4 {
  color: #92400E;
  margin: 0 0 12px 0;
  font-size: 16px;
}

.test-guide ol {
  margin: 0;
  padding-left: 20px;
  color: #92400E;
}

.test-guide li {
  margin: 6px 0;
  font-size: 14px;
  line-height: 1.4;
}

/* 반응형 디자인 */
@media (max-width: 480px) {
  .qr-generator-container {
    padding: 16px;
  }
  
  .header-section {
    margin-top: 20px;
    margin-bottom: 30px;
  }
  
  .title {
    font-size: 22px;
  }
  
  .qr-code-image {
    width: 250px;
    height: 250px;
  }
  
  .action-buttons {
    flex-direction: column;
    width: 100%;
    max-width: 300px;
  }
  
  .download-btn, .copy-btn, .reset-btn {
    width: 100%;
  }
}
</style>
