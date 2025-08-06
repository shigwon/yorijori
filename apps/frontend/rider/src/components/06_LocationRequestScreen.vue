<template>
  <div class="location-request-container">
    <!-- 메인 콘텐츠 -->
    <div class="main-content">
      <h1 class="title">위치 정보 수집 이용 동의</h1>
      
      <div class="consent-intro">
        <p><span class="brand-name">LiNKY</span>는 고객의 위치 정보를 확인하고자 합니다 아래 내용을 확인하시고 동의 여부를 결정해주세요.</p>
      </div>
      
      <div class="consent-section">
        <h2 class="section-title">[필수] 배송지 정보 수집·이용 내역</h2>
        
        <div class="consent-table">
          <div class="table-row header">
            <div class="table-cell">수집·이용 목적</div>
            <div class="table-cell">수집 항목</div>
            <div class="table-cell">보유·이용 기간</div>
          </div>
          <div class="table-row">
            <div class="table-cell">고객 위치 정보 확인</div>
            <div class="table-cell">위치정보<br>안심번호</div>
            <div class="table-cell">
              배송이 완료되는 시점까지
            </div>
          </div>
        </div>
        
        <div class="consent-disclaimer">
          <p>위의 배송지 정보 수집ㆍ이용에 대한 동의를 거부할 권리가 있습니다. 그러나 동의를 거부할 경우 간편 서비스를 이용할 수 없습니다.</p>
        </div>
      </div>
    </div>

    <!-- 액션 버튼 -->
    <div class="action-section">
      <button class="send-button" @click="handleSend">
        동 의
      </button>
    </div>
  </div>
</template>

<script setup>
import { useAppState } from '../composables/useAppState'
import { onMounted } from 'vue'

const { goToPhotoCapture, setProgressPercent, goToScanConfirmModal } = useAppState()

onMounted(() => {
  setProgressPercent(70) // Set progress to 70% on this screen
})

const handleSend = () => {
  goToPhotoCapture()
}

const handleCircleClick = () => {
  goToScanConfirmModal()
}
</script>

<style scoped>
.location-request-container {
  width: 100%;
  height: 100%;
  background: white;
  display: flex;
  flex-direction: column;
  align-items: center;
  justify-content: space-between;
  padding: 40px 24px 24px 24px;
  box-sizing: border-box;
  position: relative;
  margin-top: -40px;
}

/* 메인 콘텐츠 */
.main-content {
  flex: 1;
  display: flex;
  flex-direction: column;
  align-items: center;
  justify-content: center;
  width: 100%;
  max-width: 400px;
  margin-top: 20px;
}

.title {
  font-size: 24px;
  font-weight: 700;
  color: #1F2937;
  text-align: center;
  margin: 0 0 24px 0;
  line-height: 1.3;
}

.consent-intro {
  width: 100%;
  margin-bottom: 24px;
}

.consent-intro p {
  font-size: 16px;
  color: #374151;
  line-height: 1.6;
  margin: 0;
  text-align: justify;
}

.brand-name {
  font-size: 16px;
  font-weight: 700;
  color: #7C3AED;
}

.consent-section {
  width: 100%;
  background: #FFFFFF;
  border: 1px solid #E5E7EB;
  border-radius: 12px;
  padding: 24px;
  box-shadow: 0 2px 8px rgba(0, 0, 0, 0.05);
}

.section-title {
  font-size: 18px;
  font-weight: 700;
  color: #1F2937;
  margin: 0 0 20px 0;
  line-height: 1.4;
}

.consent-table {
  width: 100%;
  border: 1px solid #D1D5DB;
  border-radius: 8px;
  overflow: hidden;
  margin-bottom: 20px;
}

.table-row {
  display: flex;
  border-bottom: 1px solid #D1D5DB;
}

.table-row:last-child {
  border-bottom: none;
}

.table-row.header {
  background: #F9FAFB;
  font-weight: 600;
}

.table-cell {
  flex: 1;
  padding: 12px;
  font-size: 14px;
  line-height: 1.5;
  color: #374151;
  border-right: 1px solid #D1D5DB;
  display: flex;
  align-items: center;
}

.table-cell:last-child {
  border-right: none;
}

.table-row.header .table-cell {
  color: #1F2937;
  font-weight: 600;
}

.note {
  color: #6B7280;
  font-size: 13px;
  font-style: italic;
}

.disclaimer {
  color: #9CA3AF;
  font-size: 12px;
  font-style: italic;
}

.consent-disclaimer {
  background: #F3F4F6;
  border-radius: 8px;
  padding: 16px;
}

.consent-disclaimer p {
  font-size: 14px;
  color: #6B7280;
  line-height: 1.5;
  margin: 0;
  text-align: justify;
}

/* 액션 섹션 */
.action-section {
  width: 100%;
  max-width: 400px;
  padding: 0;
}

.send-button {
  width: 100%;
  height: 56px;
  background: #7C3AED;
  border: none;
  border-radius: 12px;
  color: white;
  font-size: 18px;
  font-weight: 600;
  cursor: pointer;
  transition: all 0.2s ease;
  box-shadow: 0 4px 12px rgba(124, 60, 237, 0.2);
}

.send-button:hover {
  background: #6D28D9;
  transform: translateY(-1px);
  box-shadow: 0 6px 16px rgba(124, 60, 237, 0.3);
}

.send-button:active {
  transform: translateY(0);
}

/* 모바일 반응형 */
@media (max-width: 480px) {
  .location-request-container {
    padding: 60px 16px 400px 16px;
    height: 100vh;
    box-sizing: border-box;
    margin-top: -40px;
  }
  
  .main-content {
    margin-top: 20px;
  }
  
  .title {
    font-size: 22px;
    margin-top: 0;
    margin-bottom: 30px;
  }
  
  .consent-intro p {
    font-size: 15px;
  }
  
  .consent-section {
    padding: 20px;
  }
  
  .section-title {
    font-size: 17px;
    margin-bottom: 16px;
  }
  
  .table-cell {
    padding: 10px;
    font-size: 13px;
  }
  
  .consent-disclaimer {
    padding: 14px;
  }
  
  .consent-disclaimer p {
    font-size: 13px;
  }
  
  .send-button {
    height: 52px;
    font-size: 17px;
    margin-top: 70px;
  }
}
</style>