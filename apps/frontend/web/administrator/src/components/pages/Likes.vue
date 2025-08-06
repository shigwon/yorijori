<template>
    <div class="likes-container">
      <div class="content-area">
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
              <i class="dropdown-icon">▼</i>
            </div>
          </div>
          
          <div class="rating-display">
            <div class="star-rating">
              <span class="star">⭐</span>
              <span class="rating-value">3.8</span>
            </div>
          </div>
          
          <div class="reviews-container">
            <div 
              v-for="(review, index) in reviews" 
              :key="index" 
              class="review-item"
            >
              <div class="review-content">
                <p>리뷰 내용</p>
              </div>
            </div>
          </div>
        </div>
        
        <!-- Photo Registration Section -->
        <div class="photo-registration-section">
          <div class="section-header">
            <h2>이용 고객 대비 사진 등록</h2>
            <div class="time-filter">
              <select v-model="selectedPhotoTimeFilter">
                <option value="this-week">This Week</option>
                <option value="this-month">This Month</option>
                <option value="this-year">This Year</option>
              </select>
              <i class="dropdown-icon">▼</i>
            </div>
          </div>
          
          <!-- 가로 막대 그래프 -->
          <div class="bar-chart-container">
            <div class="chart-title">별점 분포</div>
            <div class="bar-chart">
              <div class="bar-item">
                <div class="bar-label">5점</div>
                <div class="bar-wrapper">
                  <div class="bar" :style="{ width: '75%' }"></div>
                  <span class="bar-value">75%</span>
                </div>
              </div>
              <div class="bar-item">
                <div class="bar-label">4점</div>
                <div class="bar-wrapper">
                  <div class="bar" :style="{ width: '60%' }"></div>
                  <span class="bar-value">60%</span>
                </div>
              </div>
              <div class="bar-item">
                <div class="bar-label">3점</div>
                <div class="bar-wrapper">
                  <div class="bar" :style="{ width: '45%' }"></div>
                  <span class="bar-value">45%</span>
                </div>
              </div>
              <div class="bar-item">
                <div class="bar-label">2점</div>
                <div class="bar-wrapper">
                  <div class="bar" :style="{ width: '25%' }"></div>
                  <span class="bar-value">25%</span>
                </div>
              </div>
              <div class="bar-item">
                <div class="bar-label">1점</div>
                <div class="bar-wrapper">
                  <div class="bar" :style="{ width: '10%' }"></div>
                  <span class="bar-value">10%</span>
                </div>
              </div>
            </div>
          </div>
          
          <!-- 도넛 차트 -->
          <div class="donut-chart-container">
            <div class="chart-title">카테고리별 분포</div>
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
                  <span class="legend-label">Fashion</span>
                  <span class="legend-value">251K</span>
                </div>
                <div class="legend-item">
                  <div class="legend-color accessories-color"></div>
                  <span class="legend-label">Accessories</span>
                  <span class="legend-value">176K</span>
                </div>
              </div>
            </div>
          </div>
        </div>
      </div>
    </div>
  </template>
  
  <script setup>
  import { ref } from 'vue'
  
  // Reactive data
  const selectedTimeFilter = ref('this-week')
  const selectedPhotoTimeFilter = ref('this-week')
  const reviews = ref(Array(7).fill(null)) // 7개의 리뷰 아이템
  </script>
  
  <style scoped>
  .likes-container {
    min-height: 100vh;
    background-color: #0f1419;
    color: white;
  }
  
  .content-area {
    padding: 30px;
    background-color: #0f1419;
    min-height: 100vh;
    display: flex;
    flex-direction: r;
    gap: 24px;
  }
  
  /* Likes Section */
  .likes-section {
    background-color: #1a1f2e;
    border-radius: 12px;
    padding: 24px;
    box-shadow: 0 4px 12px rgba(0, 0, 0, 0.3);
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
  }
  
  .star-rating {
    display: flex;
    align-items: center;
    gap: 8px;
  }
  
  .star {
    font-size: 24px;
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
  }
  
  .review-item {
    background-color: #2a2f3e;
    border-radius: 8px;
    padding: 16px;
    transition: transform 0.2s ease;
  }
  
  .review-item:hover {
    transform: translateY(-2px);
    box-shadow: 0 4px 8px rgba(0, 0, 0, 0.2);
  }
  
  .review-content p {
    margin: 0;
    color: #8a92a6;
    font-size: 14px;
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
  }
  
  /* 가로 막대 그래프 */
  .bar-chart-container {
    margin-bottom: 20px;
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
  }
  
  .chart-container {
    display: flex;
    align-items: center;
    gap: 40px;
    margin-top: 20px;
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