import axios from 'axios'

// 관리자 전용 API 요청을 위한 axios instance
const adminClient = axios.create({
  baseURL: '/api/admin',
  timeout: 1500,
  headers: {
    'Content-Type': 'application/json',
    // 헤더 작성 예시:
    // 'X-Admin-Role': 'super_admin',
    // 'X-Request-ID': 'unique-request-id',
    // 'X-Client-Version': '1.0.0'
  }
})

// 요청 인터셉터
adminClient.interceptors.request.use(
  (config) => {
    // 세션 기반 인증 확인
    const adminSession = localStorage.getItem('adminSession') || sessionStorage.getItem('adminSession')
    const adminEmail = localStorage.getItem('adminEmail') || sessionStorage.getItem('adminEmail')
    
    if (adminSession === 'true' && adminEmail) {
      // 세션 기반 인증 헤더 추가
      config.headers['X-Admin-Session'] = 'true'
      config.headers['X-Admin-Email'] = adminEmail
      // console.log('세션 기반 API 요청:', config.method?.toUpperCase(), config.url)
    } else {
      // 세션이 없는 경우에도 요청 진행 (테스트 환경)
      console.log('세션 없이 API 요청 (테스트 모드)')
    }
    
    // 관리자 권한 헤더 추가
    config.headers['X-Admin-Role'] = 'admin'
    
    return config
  },
  (error) => {
    console.error('Admin API 요청 오류:', error)
    return Promise.reject(error)
  }
)

// 응답 인터셉터
adminClient.interceptors.response.use(
  (response) => {
    // console.log('Admin API 응답:', response.status, response.config.url)
    return response
  },
  (error) => {
    console.error('Admin API 응답 오류:', error.response?.status, error.response?.data)
    
    // 401 Unauthorized 처리 (세션 만료)
    if (error.response?.status === 401) {
      console.log('세션 만료 또는 인증 실패')
      // 세션 정보 삭제
      localStorage.removeItem('adminSession')
      localStorage.removeItem('adminEmail')
      sessionStorage.removeItem('adminSession')
      sessionStorage.removeItem('adminEmail')
      // 로그인 페이지로 리다이렉트
      window.location.href = '/'
    }
    
    // 403 Forbidden 처리
    if (error.response?.status === 403) {
      console.error('관리자 권한이 없습니다.')
    }
    
    return Promise.reject(error)
  }
)

export default adminClient 