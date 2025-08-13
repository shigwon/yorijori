<template>
    <div class="likes-container">
      <div class="content-area">
        <div class="photo-registration-section">
          <div class="section-header">
            <h2>이용 고객 대비 사진 등록</h2>
            <div class="time-filter">
              <select v-model="selectedPhotoTimeFilter">
                <option value="this-week">This Week</option>
                <option value="this-month">This Month</option>
                <option value="this-year">This Year</option>
              </select>
              <v-icon class="dropdown-icon">mdi-chevron-down</v-icon>
            </div>
          </div>
          
          <!-- 가로 막대 그래프 -->
          <div class="bar-chart-container">
            <div class="chart-title">별점 분포</div>
            <div v-if="isRatingLoading" class="loading-container">
              <div class="loading-spinner"></div>
              <p class="loading-text">별점 데이터를 불러오는 중...</p>
            </div>
            <div v-else-if="ratingError" class="error-container">
              <p class="error-text">{{ ratingError }}</p>
              <button @click="fetchRatingData" class="retry-button">다시 시도</button>
            </div>
            <div v-else class="bar-chart">
              <div 
                v-for="(count, rating) in ratingData" 
                :key="rating" 
                class="bar-item"
              >
                <div class="bar-label">{{ rating }}</div>
                <div class="bar-wrapper">
                  <div 
                    class="bar" 
                    :style="{ width: getRatingPercentage(count) + '%' }"
                  ></div>
                  <span class="bar-value">{{ count }}개</span>
                </div>
              </div>
            </div>
          </div>
          
          <!-- 도넛 차트 -->
          <div class="donut-chart-container">
            <div class="chart-title">만족도 조사 참여율</div>
            <div class="chart-container">
              <div class="donut-chart">
                <div class="chart-ring">
                  <div class="chart-segment fashion"></div>
                  <div class="chart-segment accessories"></div>
                </div>
                <div class="chart-center">
                  <div class="center-circle"></div>
                </div>
              </div>
              
              <div class="chart-legend">
                <div class="legend-item">
                  <div class="legend-color fashion-color"></div>
                  <span class="legend-label">yes</span>
                  <span class="legend-value">251K</span>
                </div>
                <div class="legend-item">
                  <div class="legend-color accessories-color"></div>
                  <span class="legend-label">no</span>
                  <span class="legend-value">176K</span>
                </div>
              </div>
            </div>
          </div>
        </div>
        <!-- Likes Section -->
        <div class="likes-section">
          <div class="section-header">
            <h2>Likes</h2>
            <div class="time-filter">
              <select v-model="selectedTimeFilter">
                <option value="this-week">This Week</option>
                <option value="this-month">This Month</option>
                <option value="this-year">This Year</option>
              </select>
              <v-icon class="dropdown-icon">mdi-chevron-down</v-icon>
            </div>
          </div>
          
          <div class="rating-display">
            <div class="star-rating">
              <v-icon class="star">mdi-star</v-icon>
              <span class="rating-value">{{ averageRating.toFixed(1) }}</span>
            </div>
          </div>
          
          <div class="reviews-container">
            <div v-if="isReviewsLoading" class="loading-container">
              <div class="loading-spinner"></div>
              <p class="loading-text">리뷰를 불러오는 중...</p>
            </div>
            <div v-else-if="reviewsError" class="error-container">
              <p class="error-text">{{ reviewsError }}</p>
              <button @click="fetchReviews" class="retry-button">다시 시도</button>
            </div>
            <div v-else>
              <div class="reviews-list">
                <div 
                  v-for="review in reviews" 
                  :key="review.id" 
                  class="review-item"
                  @click="openReviewModal(review.id)"
                >
                  <div class="review-content">
                    <div class="review-header">
                      <div class="review-rating">
                        <span class="star">⭐</span>
                        <span class="rating">{{ review.rating }}</span>
                      </div>
                      <div class="review-date">{{ formatDate(review.createdAt) }}</div>
                    </div>
                    <p class="review-text">{{ review.content }}</p>
                    <div class="review-footer">
                      <span class="order-code">주문번호: {{ review.orderCode }}</span>
                    </div>
                  </div>
                </div>
              </div>
              
              <!-- 페이지네이션 -->
              <div class="pagination">
                <button 
                  @click="changePage(currentPage - 1)" 
                  :disabled="currentPage <= 1"
                  class="page-btn"
                >
                  이전
                </button>
                <span class="page-info">{{ currentPage }} / {{ totalPages }}</span>
                <button 
                  @click="changePage(currentPage + 1)" 
                  :disabled="currentPage >= totalPages"
                  class="page-btn"
                >
                  다음
                </button>
              </div>
            </div>
          </div>
        </div>
        
        <!-- Photo Registration Section -->
        
      </div>
      
      <!-- 리뷰 상세 모달 -->
      <div v-if="showReviewModal" class="modal-overlay" @click="closeReviewModal">
        <div class="modal-content" @click.stop>
          <div class="modal-header">
            <h3>리뷰 상세</h3>
            <button @click="closeReviewModal" class="close-btn">×</button>
          </div>
          <div v-if="isDetailLoading" class="loading-container">
            <div class="loading-spinner"></div>
            <p class="loading-text">리뷰 상세를 불러오는 중...</p>
          </div>
          <div v-else-if="detailError" class="error-container">
            <p class="error-text">{{ detailError }}</p>
            <button @click="fetchReviewDetail" class="retry-button">다시 시도</button>
          </div>
          <div v-else-if="selectedReview" class="modal-body">
            <div class="detail-rating">
              <span class="star">⭐</span>
              <span class="rating">{{ selectedReview.rating }}</span>
            </div>
            <div class="detail-content">
              <p>{{ selectedReview.content }}</p>
            </div>
            <div class="detail-info">
              <p><strong>주문번호:</strong> {{ selectedReview.orderCode }}</p>
              <p><strong>작성일:</strong> {{ formatDate(selectedReview.createdAt) }}</p>
            </div>
          </div>
        </div>
      </div>
    </div>
  </template>
  
  <script setup>
  import { ref, onMounted, computed } from 'vue'
  import { getReviewRating, getReviews, getReview } from '../../api/examples.js'
  
  // Reactive data
  const selectedTimeFilter = ref('this-week')
  const selectedPhotoTimeFilter = ref('this-week')
  
  // 별점 데이터
  const ratingData = ref({})
  const averageRating = ref(0)
  const isRatingLoading = ref(false)
  const ratingError = ref(null)
  
  // 리뷰 데이터
  const reviews = ref([])
  const isReviewsLoading = ref(false)
  const reviewsError = ref(null)
  const currentPage = ref(1)
  const totalPages = ref(1)
  const totalCount = ref(0)
  
  // 모달 관련
  const showReviewModal = ref(false)
  const selectedReview = ref(null)
  const isDetailLoading = ref(false)
  const detailError = ref(null)
  const selectedReviewId = ref(null)
  
  // 별점 데이터 가져오기
  const fetchRatingData = async () => {
    try {
      isRatingLoading.value = true
      ratingError.value = null
      
      const response = await getReviewRating()
      
      if (response.result === 'success' && response.data) {
        ratingData.value = response.data.ratingRangeCounts
        averageRating.value = response.data.averageRating
      } else {
        throw new Error('별점 데이터를 가져올 수 없습니다.')
      }
    } catch (err) {
      console.error('별점 데이터 로드 오류:', err)
      ratingError.value = err.message || '별점 데이터 로드에 실패했습니다.'
      
      // 기본 데이터 설정
      ratingData.value = {
        '5점': 6,
        '4점': 6,
        '3점': 2,
        '2점': 1,
        '1점': 0
      }
      averageRating.value = 4.1
    } finally {
      isRatingLoading.value = false
    }
  }
  
  // 리뷰 목록 가져오기
  const fetchReviews = async () => {
    try {
      isReviewsLoading.value = true
      reviewsError.value = null
      
      const response = await getReviews()
      
      if (response.result === 'success' && response.data) {
        reviews.value = response.data.reviews
        totalCount.value = response.data.totalCount
        totalPages.value = response.data.totalPages
        currentPage.value = response.data.currentPage
      } else {
        throw new Error('리뷰 목록을 가져올 수 없습니다.')
      }
    } catch (err) {
      console.error('리뷰 목록 로드 오류:', err)
      reviewsError.value = err.message || '리뷰 목록 로드에 실패했습니다.'
      
      // 기본 데이터 설정
      reviews.value = [
        {
          id: 7,
          orderCode: "10",
          rating: 5,
          content: "정말 혁신적인 서비스네요! 배달도 빠르고 정확했습니다.",
          createdAt: "2025-08-05T21:15:00"
        },
        {
          id: 12,
          orderCode: "15",
          rating: 3,
          content: "배달은 됐지만 로봇이 약간 소음이 있었어요. 그래도 편리합니다.",
          createdAt: "2025-08-02T15:30:00"
        }
      ]
      totalCount.value = 2
      totalPages.value = 1
      currentPage.value = 1
    } finally {
      isReviewsLoading.value = false
    }
  }
  
  // 리뷰 상세 가져오기
  const fetchReviewDetail = async () => {
    if (!selectedReviewId.value) return
    
    try {
      isDetailLoading.value = true
      detailError.value = null
      
      const response = await getReview(selectedReviewId.value)
      
      if (response.result === 'success' && response.data) {
        selectedReview.value = response.data
      } else {
        throw new Error('리뷰 상세를 가져올 수 없습니다.')
      }
    } catch (err) {
      console.error('리뷰 상세 로드 오류:', err)
      detailError.value = err.message || '리뷰 상세 로드에 실패했습니다.'
    } finally {
      isDetailLoading.value = false
    }
  }
  
  // 별점 퍼센트 계산
  const getRatingPercentage = (count) => {
    const total = Object.values(ratingData.value).reduce((sum, val) => sum + val, 0)
    return total > 0 ? Math.round((count / total) * 100) : 0
  }
  
  // 날짜 포맷팅
  const formatDate = (dateString) => {
    const date = new Date(dateString)
    return date.toLocaleDateString('ko-KR', {
      year: 'numeric',
      month: '2-digit',
      day: '2-digit',
      hour: '2-digit',
      minute: '2-digit'
    })
  }
  
  // 페이지 변경
  const changePage = (page) => {
    if (page >= 1 && page <= totalPages.value) {
      currentPage.value = page
      fetchReviews()
    }
  }
  
  // 모달 열기
  const openReviewModal = (reviewId) => {
    selectedReviewId.value = reviewId
    showReviewModal.value = true
    fetchReviewDetail()
  }
  
  // 모달 닫기
  const closeReviewModal = () => {
    showReviewModal.value = false
    selectedReview.value = null
    selectedReviewId.value = null
    detailError.value = null
  }
  
  // 컴포넌트 마운트 시 데이터 로드
  onMounted(() => {
    fetchRatingData()
    fetchReviews()
  })
  </script>
  
  <style scoped>
  .likes-container {
    min-height: 100vh;
    min-width: 100%;
    
    color: white;
  }
  
  .content-area {
    padding-top: 20px;
    min-height: 100vh;
    display: flex;
    flex-direction: row;
    gap: 24px;
    width: 100%;
    height: 100vh;
    overflow: hidden;
    box-sizing: border-box;
  }
  
  /* Likes Section */
  .likes-section {
    background-color: #1a1f2e;
    border-radius: 12px;
    padding: 24px;
    box-shadow: 0 4px 12px rgba(0, 0, 0, 0.3);
    flex: 1;
    min-width: 0;
    height: 100%;
    display: flex;
    flex-direction: column;
    overflow: hidden;
    box-sizing: border-box;
  }
  
  .section-header {
    display: flex;
    justify-content: space-between;
    align-items: center;
    margin-bottom: 20px;
  }
  
  .section-header h2 {
    font-size: 20px;
    font-weight: 600;
    margin: 0;
    color: #ffffff;
  }
  
  .time-filter {
    position: relative;
    display: flex;
    align-items: center;
  }
  
  .time-filter select {
    background-color: #2a2f3e;
    border: 1px solid #3a57e8;
    color: white;
    padding: 8px 12px;
    border-radius: 6px;
    font-size: 14px;
    cursor: pointer;
    appearance: none;
    padding-right: 30px;
  }
  
  .dropdown-icon {
    position: absolute;
    right: 10px;
    font-size: 12px;
    color: #8a92a6;
    pointer-events: none;
  }
  
  .rating-display {
    margin-bottom: 24px;
    display: flex;
    justify-content: center;
  }
  
  .star-rating {
    display: flex;
    align-items: center;
    gap: 8px;
    justify-content: center;
  }
  
  .star {
    font-size: 24px;
    color: #ffd700;
  }
  
  .rating-value {
    font-size: 20px;
    font-weight: 600;
    color: #ffd700;
  }
  
  .reviews-container {
    display: flex;
    flex-direction: column;
    gap: 12px;
    flex: 1;
    overflow: hidden;
    min-height: 0;
  }
  
  .reviews-list {
    flex: 1;
    overflow-y: auto;
    padding-right: 8px;
    min-height: 0;
    max-height: calc(100vh - 300px);
  }
  
  .review-item {
    background-color: #2a2f3e;
    border-radius: 8px;
    padding: 16px;
    transition: all 0.2s ease;
    cursor: pointer;
  }
  
  .review-item:hover {
    transform: translateY(-2px);
    box-shadow: 0 4px 8px rgba(0, 0, 0, 0.2);
    background-color: #3a3f4e;
  }
  
  .review-content p {
    margin: 0;
    color: #8a92a6;
    font-size: 14px;
  }
  
  .review-header {
    display: flex;
    justify-content: space-between;
    align-items: center;
    margin-bottom: 8px;
  }
  
  .review-rating {
    display: flex;
    align-items: center;
    gap: 4px;
  }
  
  .review-rating .star {
    font-size: 16px;
  }
  
  .review-rating .rating {
    font-size: 14px;
    font-weight: 600;
    color: #ffd700;
  }
  
  .review-date {
    font-size: 12px;
    color: #8a92a6;
  }
  
  .review-text {
    margin: 8px 0;
    color: #ffffff;
    font-size: 14px;
    line-height: 1.4;
  }
  
  .review-footer {
    margin-top: 8px;
  }
  
  .order-code {
    font-size: 12px;
    color: #8a92a6;
  }
  
  /* 페이지네이션 */
  .pagination {
    display: flex;
    justify-content: center;
    align-items: center;
    gap: 16px;
    margin-top: 16px;
    padding: 12px 0;
    flex-shrink: 0;
    border-top: 1px solid #2a2f3e;
    padding-top: 16px;
    background-color: #1a1f2e;
  }
  
  .page-btn {
    background-color: #2a2f3e;
    border: 1px solid #3a57e8;
    color: white;
    padding: 8px 16px;
    border-radius: 6px;
    cursor: pointer;
    font-size: 14px;
    transition: all 0.2s ease;
  }
  
  .page-btn:hover:not(:disabled) {
    background-color: #3a57e8;
  }
  
  .page-btn:disabled {
    opacity: 0.5;
    cursor: not-allowed;
  }
  
  .page-info {
    font-size: 14px;
    color: #8a92a6;
    font-weight: 500;
  }
  
  /* 로딩 및 에러 상태 */
  .loading-container {
    display: flex;
    flex-direction: column;
    align-items: center;
    justify-content: center;
    padding: 40px 20px;
    gap: 16px;
  }
  
  .loading-spinner {
    width: 32px;
    height: 32px;
    border: 3px solid #2a2f3e;
    border-top: 3px solid #3a57e8;
    border-radius: 50%;
    animation: spin 1s linear infinite;
  }
  
  @keyframes spin {
    0% { transform: rotate(0deg); }
    100% { transform: rotate(360deg); }
  }
  
  .loading-text {
    color: #8a92a6;
    font-size: 14px;
    margin: 0;
  }
  
  .error-container {
    display: flex;
    flex-direction: column;
    align-items: center;
    justify-content: center;
    padding: 40px 20px;
    gap: 16px;
  }
  
  .error-text {
    color: #ef4444;
    font-size: 14px;
    text-align: center;
    margin: 0;
  }
  
  .retry-button {
    background-color: #3a57e8;
    border: none;
    color: white;
    padding: 8px 16px;
    border-radius: 6px;
    cursor: pointer;
    font-size: 14px;
    transition: background-color 0.2s ease;
  }
  
  .retry-button:hover {
    background-color: #2563eb;
  }
  
  /* 모달 스타일 */
  .modal-overlay {
    position: fixed;
    top: 0;
    left: 0;
    right: 0;
    bottom: 0;
    background-color: rgba(0, 0, 0, 0.7);
    display: flex;
    align-items: center;
    justify-content: center;
    z-index: 1000;
  }
  
  .modal-content {
    background-color: #1a1f2e;
    border-radius: 12px;
    padding: 0;
    max-width: 500px;
    width: 90%;
    max-height: 80vh;
    overflow-y: auto;
    box-shadow: 0 8px 32px rgba(0, 0, 0, 0.5);
  }
  
  .modal-header {
    display: flex;
    justify-content: space-between;
    align-items: center;
    padding: 20px 24px;
    border-bottom: 1px solid #2a2f3e;
  }
  
  .modal-header h3 {
    margin: 0;
    color: #ffffff;
    font-size: 18px;
    font-weight: 600;
  }
  
  .close-btn {
    background: none;
    border: none;
    color: #8a92a6;
    font-size: 24px;
    cursor: pointer;
    padding: 0;
    width: 32px;
    height: 32px;
    display: flex;
    align-items: center;
    justify-content: center;
    border-radius: 50%;
    transition: all 0.2s ease;
  }
  
  .close-btn:hover {
    background-color: #2a2f3e;
    color: #ffffff;
  }
  
  .modal-body {
    padding: 24px;
  }
  
  .detail-rating {
    display: flex;
    align-items: center;
    gap: 8px;
    margin-bottom: 20px;
  }
  
  .detail-rating .star {
    font-size: 24px;
  }
  
  .detail-rating .rating {
    font-size: 20px;
    font-weight: 600;
    color: #ffd700;
  }
  
  .detail-content {
    margin-bottom: 20px;
  }
  
  .detail-content p {
    color: #ffffff;
    font-size: 16px;
    line-height: 1.6;
    margin: 0;
  }
  
  .detail-info {
    border-top: 1px solid #2a2f3e;
    padding-top: 16px;
  }
  
  .detail-info p {
    color: #8a92a6;
    font-size: 14px;
    margin: 8px 0;
  }
  
  .detail-info strong {
    color: #ffffff;
  }
  
  /* Photo Registration Section */
  .photo-registration-section {
    background-color: #1a1f2e;
    border-radius: 12px;
    padding: 24px;
    box-shadow: 0 4px 12px rgba(0, 0, 0, 0.3);
    display: flex;
    flex-direction: column;
    gap: 30px;
    flex: 1;
    min-width: 0;
    height: 100%;
    overflow: hidden;
    box-sizing: border-box;
  }
  
  /* 가로 막대 그래프 */
  .bar-chart-container {
    margin-bottom: 20px;
    flex: 1;
    display: flex;
    flex-direction: column;
  }
  
  .chart-title {
    font-size: 18px;
    font-weight: 600;
    color: #ffffff;
    margin-bottom: 20px;
  }
  
  .bar-chart {
    display: flex;
    flex-direction: column;
    gap: 15px;
    flex: 1;
  }
  
  .bar-item {
    display: flex;
    align-items: center;
    gap: 15px;
  }
  
  .bar-label {
    min-width: 40px;
    font-size: 14px;
    color: #8a92a6;
    font-weight: 500;
  }
  
  .bar-wrapper {
    flex: 1;
    display: flex;
    align-items: center;
    gap: 10px;
    position: relative;
  }
  
  .bar {
    height: 20px;
    background: linear-gradient(90deg, #3a57e8 0%, #4774b6 100%);
    border-radius: 10px;
    transition: width 0.3s ease;
    position: relative;
  }
  
  .bar-value {
    font-size: 12px;
    color: #8a92a6;
    font-weight: 500;
    min-width: 35px;
  }
  
  /* 도넛 차트 */
  .donut-chart-container {
    margin-top: 20px;
    flex: 1;
    display: flex;
    flex-direction: column;
  }
  
  .chart-container {
    display: flex;
    align-items: center;
    gap: 40px;
    margin-top: 20px;
    flex: 1;
  }
  
  .donut-chart {
    position: relative;
    width: 200px;
    height: 200px;
  }
  
  .chart-ring {
    position: relative;
    width: 100%;
    height: 100%;
    border-radius: 50%;
    background: conic-gradient(
      #3a57e8 0deg 180deg,
      #4774b6 180deg 360deg
    );
  }
  
  .chart-center {
    position: absolute;
    top: 50%;
    left: 50%;
    transform: translate(-50%, -50%);
  }
  
  .center-circle {
    width: 80px;
    height: 80px;
    background-color: #2a2f3e;
    border-radius: 50%;
  }
  
  .chart-legend {
    display: flex;
    flex-direction: column;
    gap: 16px;
  }
  
  .legend-item {
    display: flex;
    align-items: center;
    gap: 12px;
  }
  
  .legend-color {
    width: 16px;
    height: 16px;
    border-radius: 50%;
  }
  
  .fashion-color {
    background-color: #3a57e8;
  }
  
  .accessories-color {
    background-color: #4774b6;
  }
  
  .legend-label {
    font-size: 14px;
    color: #ffffff;
    min-width: 80px;
  }
  
  .legend-value {
    font-size: 14px;
    font-weight: 600;
    color: #3a57e8;
  }
  
  /* 반응형 디자인 */
  @media (max-width: 768px) {
    .content-area {
      padding: 20px;
      flex-direction: column;
      height: auto;
      overflow: visible;
    }
    
    .likes-section,
    .photo-registration-section {
      flex: none;
      width: 100%;
      height: auto;
      overflow: visible;
    }
    
    .section-header {
      flex-direction: column;
      align-items: flex-start;
      gap: 12px;
    }
    
    .chart-container {
      flex-direction: column;
      gap: 24px;
    }
    
    .donut-chart {
      width: 150px;
      height: 150px;
    }
    
    .center-circle {
      width: 60px;
      height: 60px;
    }
    
    .bar-item {
      flex-direction: column;
      align-items: flex-start;
      gap: 8px;
    }
    
    .bar-wrapper {
      width: 100%;
    }
    
    .reviews-list {
      max-height: 400px;
    }
  }
  
  @media (max-width: 480px) {
    .content-area {
      padding: 15px;
    }
    
    .likes-section,
    .photo-registration-section {
      padding: 16px;
    }
    
    .donut-chart {
      width: 120px;
      height: 120px;
    }
    
    .center-circle {
      width: 50px;
      height: 50px;
    }
    
    .bar-label {
      min-width: 30px;
      font-size: 12px;
    }
    
    .bar-value {
      font-size: 11px;
    }
  }
  </style> 