package com.linky.config.database;

import com.zaxxer.hikari.HikariConfig;
import com.zaxxer.hikari.HikariDataSource;
import org.apache.ibatis.session.SqlSessionFactory;
import org.mybatis.spring.SqlSessionFactoryBean;
import org.mybatis.spring.annotation.MapperScan;
import org.springframework.beans.factory.annotation.Qualifier;
import org.springframework.beans.factory.annotation.Value;
import org.springframework.context.annotation.Bean;
import org.springframework.context.annotation.Configuration;
import org.springframework.context.annotation.Primary;
import org.springframework.core.io.support.PathMatchingResourcePatternResolver;
import org.springframework.jdbc.datasource.DataSourceTransactionManager;
import org.springframework.transaction.PlatformTransactionManager;

import javax.sql.DataSource;

@Configuration
@MapperScan(
        basePackages = {
                "com.linky.api.admin.repository",     // 로그인용
                "com.linky.api.order.repository",
                "com.linky.api.review.repository",
                "com.linky.api.robot.repository.mybatis"
        },
        sqlSessionFactoryRef = "mysqlSqlSessionFactory"
        // JPA Repository는 자동으로 제외됨 (@Repository vs @Mapper 구분)
)
public class MySQLConfig {

    @Value("${spring.datasource.mysql.url}")
    private String mysqlUrl;

    @Value("${spring.datasource.mysql.username}")
    private String mysqlUsername;

    @Value("${spring.datasource.mysql.password}")
    private String mysqlPassword;

    @Value("${spring.datasource.mysql.driver-class-name}")
    private String mysqlDriverClassName;

    @Bean(name = "mysqlDataSource")
    @Primary  // 기본 데이터소스로 설정
    public DataSource mysqlDataSource() {
        HikariConfig config = new HikariConfig();
        config.setJdbcUrl(mysqlUrl);
        config.setUsername(mysqlUsername);
        config.setPassword(mysqlPassword);
        config.setDriverClassName(mysqlDriverClassName);

        // Connection Pool 설정
        config.setMinimumIdle(3);
        config.setMaximumPoolSize(15);
        config.setConnectionTimeout(30000);
        config.setIdleTimeout(300000);
        config.setMaxLifetime(1200000);
        config.setPoolName("MySQL-HikariPool");

        return new HikariDataSource(config);
    }

    @Bean(name = "mysqlSqlSessionFactory")
    @Primary  // 기본 SqlSessionFactory로 설정
    public SqlSessionFactory mysqlSqlSessionFactory(
            @Qualifier("mysqlDataSource") DataSource dataSource) throws Exception {

        SqlSessionFactoryBean sessionFactory = new SqlSessionFactoryBean();
        sessionFactory.setDataSource(dataSource);
        sessionFactory.setMapperLocations(
                new PathMatchingResourcePatternResolver().getResources("classpath:/mappers/**/*.xml"));
        sessionFactory.setTypeAliasesPackage("com.linky.api.*.entity");

        return sessionFactory.getObject();
    }

    @Bean(name = "mysqlTransactionManager")
    @Primary  // 기본 트랜잭션 매니저로 설정
    public PlatformTransactionManager mysqlTransactionManager(
            @Qualifier("mysqlDataSource") DataSource dataSource) {
        return new DataSourceTransactionManager(dataSource);
    }
}