#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "SteeringAgent.generated.h"

UENUM(BlueprintType)
enum class ESteeringBehavior : uint8
{
    Seek UMETA(DisplayName = "Seek"),
    Flee UMETA(DisplayName = "Flee"),
    Pursue UMETA(DisplayName = "Pursue"),
    Evade UMETA(DisplayName = "Evade"),
    Arrive UMETA(DisplayName = "Arrive"),
    Wander UMETA(DisplayName = "Wander"),
    PathFollowing UMETA(DisplayName = "Path Following"),
    Separation UMETA(DisplayName = "Separation"),
    CollisionAvoidance UMETA(DisplayName = "Collision Avoidance"),
    ObstacleAvoidance UMETA(DisplayName = "Obstacle Avoidance"),
    WallAvoidance UMETA(DisplayName = "Wall Avoidance"),
    Combined UMETA(DisplayName = "Combined (Task 3)")
};

UCLASS()
class STEERINGBEHAVIORS_API ASteeringAgent : public AActor
{
    GENERATED_BODY()

public:
    ASteeringAgent();
    virtual void Tick(float DeltaTime) override;

protected:
    virtual void BeginPlay() override;

    // All steering movement is on the XY plane.
    FVector Velocity = FVector::ZeroVector;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Steering")
    float MaxSpeed = 300.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Steering")
    float MaxForce = 150.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Steering")
    ESteeringBehavior Behavior = ESteeringBehavior::Seek;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Steering")
    AActor* TargetActor = nullptr;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Steering")
    float SlowRadius = 400.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Wander")
    float WanderDistance = 300.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Wander")
    float WanderRadius = 150.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Wander")
    float WanderJitter = 1.0f;

    float WanderAngle = 0.0f;

    // Drag PathPoint_1, PathPoint_2, PathPoint_3, PathPoint_4 into this array.
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Path Following")
    TArray<AActor*> PathPoints;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Path Following")
    float WaypointRadius = 150.0f;

    int32 CurrentPathIndex = 0;

    // Add the other BP_SteeringAgent actors here (not this agent).
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Avoidance")
    TArray<ASteeringAgent*> OtherAgents;

    // Add cubes or other static obstacle actors here.
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Avoidance")
    TArray<AActor*> Obstacles;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Avoidance")
    float SeparationRadius = 250.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Avoidance")
    float CollisionLookAhead = 1.5f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Avoidance")
    float AgentRadius = 70.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Avoidance")
    float ObstacleLookAhead = 350.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Avoidance")
    float ObstacleExtraRadius = 100.0f;

    // Platform is centered at X=0, Y=0, with a 2000 x 2000 top.
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Walls")
    float WallLimitX = 1000.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Walls")
    float WallLimitY = 1000.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Walls")
    float WallMargin = 80.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Debug")
    bool bDrawDebug = true;

    FVector CalculateSteering(FVector DesiredVelocity);
    FVector Seek(FVector Target);
    FVector Flee(FVector Target);
    FVector Pursue(AActor* Target);
    FVector Evade(AActor* Target);
    FVector Arrive(FVector Target);
    FVector Wander(float DeltaTime);

    FVector PathFollowing();
    FVector Separation();
    FVector CollisionAvoidance();
    FVector ObstacleAvoidance();
    FVector WallAvoidance();
    FVector Combined();

    FVector GetFlatLocation() const;
    FVector GetFlatDirection() const;
    void DrawDebugInfo();
};
